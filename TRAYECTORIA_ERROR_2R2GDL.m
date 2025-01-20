% robot_2R_trayectoria_perturbaciones.m
clc;
clear;
close all;

% Parámetros del robot
l1 = 1;  
l2 = 1;  
m1 = 1;  
m2 = 1;  
g = 9.81;  

% Trayectoria deseada en el espacio cartesiano
t_total = 10; % Duración total de la trayectoria
t_points = linspace(0, t_total, 500); % Puntos de tiempo
x_d = linspace(0.5, 1.0, length(t_points)); % Trayectoria lineal en x
y_d = linspace(0.5, 1.0, length(t_points)); % Trayectoria lineal en y

% Calcular los ángulos deseados (\theta_1, \theta_2) usando cinemática inversa
theta_d = zeros(length(t_points), 2);
for i = 1:length(t_points)
    % Cinemática inversa
    c2 = (x_d(i)^2 + y_d(i)^2 - l1^2 - l2^2) / (2 * l1 * l2);
    s2 = sqrt(1 - c2^2); % Asumimos solución positiva
    theta2 = atan2(s2, c2);
    theta1 = atan2(y_d(i), x_d(i)) - atan2(l2 * s2, l1 + l2 * c2);
    theta_d(i, :) = [theta1, theta2];
end

% Ganancias del controlador PID
Kp = diag([300, 300]); % Ganancia proporcional
Kd = diag([100, 100]); % Ganancia derivativa
Ki = diag([50, 50]);   % Ganancia integral

% Configuración de las fuerzas de perturbación
perturbations = [5, 15, 30]; % Fuerza baja, media y alta
results = struct();

for p_idx = 1:length(perturbations)
    % Fuerza de perturbación actual
    perturb_force = perturbations(p_idx);
    
    % Estado inicial
    x0 = [theta_d(1, 1); theta_d(1, 2); 0; 0; 0; 0]; % Estado inicial
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    
    % Simulación
    [t, x] = ode45(@(t, x) robot_dynamics_pid_perturbation(t, x, theta_d, t_points, Kp, Kd, Ki, l1, l2, m1, m2, g, perturb_force), t_points, x0, options);
    
    % Posición del extremo del robot
    x_actual = zeros(length(t), 2); % Posición actual en [x, y]
    for i = 1:length(t)
        theta1 = x(i, 1);
        theta2 = x(i, 2);
        x_actual(i, 1) = l1 * cos(theta1) + l2 * cos(theta1 + theta2); % x
        x_actual(i, 2) = l1 * sin(theta1) + l2 * sin(theta1 + theta2); % y
    end
    
    % Error de seguimiento
    error_seguimiento = sqrt((x_actual(:, 1) - x_d').^2 + (x_actual(:, 2) - y_d').^2);
    
    % Guardar resultados
    results(p_idx).t = t;
    results(p_idx).x_actual = x_actual;
    results(p_idx).error = error_seguimiento;
    results(p_idx).perturb_force = perturb_force;
end

% Gráficas
for p_idx = 1:length(perturbations)
    % Extraer datos
    t = results(p_idx).t;
    x_actual = results(p_idx).x_actual;
    error_seguimiento = results(p_idx).error;
    perturb_force = results(p_idx).perturb_force;
    
    % Gráfica de trayectoria
    figure;
    plot(x_d, y_d, 'g--', 'LineWidth', 1.5); % Trayectoria deseada
    hold on;
    plot(x_actual(:, 1), x_actual(:, 2), 'b-', 'LineWidth', 1.5); % Trayectoria seguida
    xlabel('x [m]');
    ylabel('y [m]');
    legend('Trayectoria deseada', 'Trayectoria seguida');
    title(['Trayectoria del extremo del robot (Fuerza de perturbación = ', num2str(perturb_force), ' N)']);
    grid on;
    
    % Gráfica del error de seguimiento
    figure;
    plot(t, error_seguimiento, 'r-', 'LineWidth', 1.5);
    xlabel('Tiempo [s]');
    ylabel('Error de seguimiento [m]');
    title(['Error de seguimiento (Fuerza de perturbación = ', num2str(perturb_force), ' N)']);
    grid on;
end

% Función de dinámica del robot con perturbaciones
function dx = robot_dynamics_pid_perturbation(t, x, theta_d, t_points, Kp, Kd, Ki, l1, l2, m1, m2, g, perturb_force)
    % Variables de estado
    theta1 = x(1);
    theta2 = x(2);
    theta_dot1 = x(3);
    theta_dot2 = x(4);
    integral_error1 = x(5);
    integral_error2 = x(6);
    
    % Interpolación de los ángulos deseados
    theta_desired = interp1(t_points, theta_d, t);
    theta1_d = theta_desired(1);
    theta2_d = theta_desired(2);
    
    % Matriz de inercia
    M11 = m1*l1^2/3 + m2*(l1^2 + l2^2/3 + l1*l2*cos(theta2));
    M12 = m2*(l2^2/3 + l1*l2*cos(theta2)/2);
    M21 = M12;
    M22 = m2*l2^2/3;
    M = [M11, M12; M21, M22];
    
    % Matriz de Coriolis y centrífuga
    C1 = -m2*l1*l2*sin(theta2) * theta_dot2 * (2*theta_dot1 + theta_dot2);
    C2 = m2*l1*l2*sin(theta2)*theta_dot1^2;
    C = [C1; C2];
    
    % Gravedad
    G1 = (m1*l1/2 + m2*l1)*g*cos(theta1) + m2*l2*g*cos(theta1 + theta2)/2;
    G2 = m2*l2*g*cos(theta1 + theta2)/2;
    G = [G1; G2];
    
    % Perturbación externa
    if t >= 2 && t <= 4
        perturbation = [perturb_force; perturb_force];
    else
        perturbation = [0; 0];
    end
    
    % Control PID
    theta = [theta1; theta2];
    theta_dot = [theta_dot1; theta_dot2];
    error = theta - [theta1_d; theta2_d];
    integral_error = [integral_error1; integral_error2] + error * 0.01; % Integral acumulada
    tau_PD = -Kp*error - Kd*theta_dot; % Proporcional-Derivativo
    tau_I = -Ki * integral_error;      % Integral
    tau = tau_PD + tau_I + G + C + perturbation; % Control total
    
    % Dinámica del sistema
    theta_ddot = M \ tau;
    dx = [theta_dot; theta_ddot; error]; % Derivadas de estado
end
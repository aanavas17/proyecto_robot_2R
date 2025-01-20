% robot_2R_control_PID_graficos_y_animacion.m
clc;
clear;
close all;

% Parámetros del robot
l1 = 1;  
l2 = 1;  
m1 = 1;  
m2 = 1;  
g = 9.81;  

% Posición de equilibrio (ángulos objetivo)
x_eq = [pi/4; pi/6]; % Ángulos de equilibrio deseados

% Ganancias del controlador PID
Kp = diag([300, 300]); % Ganancia proporcional
Kd = diag([100, 100]); % Ganancia derivativa
Ki = diag([50, 50]);   % Ganancia integral

% Tiempo de simulación
tspan = linspace(0, 10, 1000); % Más puntos para suavizar las curvas

% Estado inicial (posición de equilibrio)
x0 = [x_eq(1); x_eq(2); 0; 0; 0; 0]; % Ángulos, velocidades, y error acumulado

% Ajuste del solver para mayor precisión
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
[t, x] = ode45(@(t, x) robot_dynamics_pid(t, x, x_eq, Kp, Kd, Ki, l1, l2, m1, m2, g), tspan, x0, options);

% 1. Gráfica de ángulos del robot
figure; % Crear una nueva ventana
plot(t, x(:, 1), 'b-', 'LineWidth', 1.5); % Línea azul sólida para \theta_1
hold on;
plot(t, x(:, 2), 'r-', 'LineWidth', 1.5); % Línea roja sólida para \theta_2
grid on;
ylim([0.4, 0.9]); % Ajuste del rango del eje Y

% Etiquetas de los ejes con text
text(mean(xlim), min(ylim) - 0.02, 'Tiempo (s)', 'FontSize', 12, 'HorizontalAlignment', 'center'); % Etiqueta eje X
text(min(xlim) - 0.1, mean(ylim), 'Ángulos (rad)', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Rotation', 90); % Etiqueta eje Y

% Título manual con text
x_pos = mean(xlim); % Centro del gráfico
y_pos = max(ylim) + 0.02; % Encima del gráfico
text(x_pos, y_pos, 'Ángulos del robot', 'FontSize', 14, 'HorizontalAlignment', 'center');

pause(0.5); % Pausa para asegurarse de que el gráfico se renderice

% 2. Gráfica de velocidades angulares
figure; % Crear una nueva ventana
plot(t, x(:, 3), 'b-', 'LineWidth', 1.5); % Velocidad angular \theta_1
hold on;
plot(t, x(:, 4), 'r-', 'LineWidth', 1.5); % Velocidad angular \theta_2
grid on;
ylim([-1, 1]); % Ajuste del rango del eje Y

% Etiquetas de los ejes con text
text(mean(xlim), min(ylim) - 0.02, 'Tiempo (s)', 'FontSize', 12, 'HorizontalAlignment', 'center'); % Etiqueta eje X
text(min(xlim) - 0.1, mean(ylim), 'Velocidades angulares (rad/s)', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Rotation', 90); % Etiqueta eje Y

% Título manual con text
x_pos = mean(xlim); % Centro del gráfico
y_pos = max(ylim) + 0.1; % Encima del gráfico
text(x_pos, y_pos, 'Velocidades angulares del robot', 'FontSize', 14, 'HorizontalAlignment', 'center');

pause(0.5); % Pausa para asegurarse de que el gráfico se renderice

% 3. Animación del robot
figure_handle = figure; % Crear una nueva ventana para la animación
for i = 1:10:length(t) % Reduce la frecuencia de actualización para mejorar el rendimiento
    % Calcula las posiciones de los eslabones
    theta1 = x(i, 1);
    theta2 = x(i, 2);
    x1 = l1 * cos(theta1);
    y1 = l1 * sin(theta1);
    x2 = x1 + l2 * cos(theta1 + theta2);
    y2 = y1 + l2 * sin(theta1 + theta2);
    
    % Dibuja el robot
    figure(figure_handle); % Asegura que solo se actualice la figura de animación
    plot([0, x1, x2], [0, y1, y2], '-o', 'LineWidth', 2);
    axis equal;
    xlim([-2, 2]);
    ylim([-2, 2]);
    grid on;
    text(0, 2.2, 'Animación del Robot 2R', 'FontSize', 14, 'HorizontalAlignment', 'center'); % Título manual con text
    drawnow;
end

% Función de dinámica del robot
function dx = robot_dynamics_pid(t, x, x_eq, Kp, Kd, Ki, l1, l2, m1, m2, g)
    % Variables de estado
    theta1 = x(1);
    theta2 = x(2);
    theta_dot1 = x(3);
    theta_dot2 = x(4);
    integral_error1 = x(5);
    integral_error2 = x(6);
    
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
    
    % Perturbación más significativa
    if t >= 2 && t <= 5
        perturbation = [15; 10]; % Perturbación en ambas articulaciones
    else
        perturbation = [0; 0];
    end
    
    % Control PID+ (compensación de dinámica completa)
    theta = [theta1; theta2];
    theta_dot = [theta_dot1; theta_dot2];
    error = theta - x_eq;
    integral_error = [integral_error1; integral_error2] + error * 0.01; % Integral acumulada
    
    tau_PD = -Kp*error - Kd*theta_dot; % Control proporcional-derivativo
    tau_I = -Ki * integral_error;      % Control integral
    tau_feedforward = G + C;           % Compensación de gravedad y Coriolis
    tau = tau_PD + tau_I + tau_feedforward + perturbation; % Control total
    
    % Dinámica del sistema
    theta_ddot = M \ (tau);
    dx = [theta_dot; theta_ddot; error]; % Incluir el error acumulativo para la integral
end
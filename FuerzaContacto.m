clc;
clear;
close all;

% Parámetros del robot
l1 = 1; l2 = 1; % Longitudes
m1 = 1; m2 = 1; % Masas
g = 9.81;        % Gravedad
x_eq = [pi/4; pi/6]; % Ángulos de equilibrio

% Ganancias del controlador PID
Kp = diag([300, 300]);
Kd = diag([100, 100]);
Ki = diag([50, 50]);

% Tiempo de simulación
tspan = linspace(0, 10, 1000);

% Fuerzas de perturbación y amortiguamientos
perturb_forces = [5, 15, 30];
D_values = [5, 15, 30]; % Amortiguamiento

% Almacenar resultados
force_results = struct();

for i = 1:length(D_values)
    D = D_values(i);
    fprintf("Simulando para D = %.1f\n", D);
    
    for j = 1:length(perturb_forces)
        F_ext = perturb_forces(j);
        fprintf("  Fuerza de perturbación = %.1f N\n", F_ext);
        
        % Estado inicial
        x0 = [x_eq(1); x_eq(2); 0; 0; 0; 0];

        % Solver ODE
        options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
        [t, x] = ode45(@(t, x) dynamics(t, x, x_eq, Kp, Kd, Ki, l1, l2, m1, m2, g, D, F_ext), tspan, x0, options);

        % Fuerza de contacto acumulada
        F_contact = calculate_contact_force(x, l1, l2, m1, m2, g, D);

        % Guardar resultados
        force_results(i, j).D = D;
        force_results(i, j).F_ext = F_ext;
        force_results(i, j).time = t;
        force_results(i, j).F_contact = F_contact;
    end
end

% Gráficas de fuerza de contacto para diferentes valores de D
figure;
for i = 1:length(D_values)
    subplot(1, length(D_values), i);
    hold on;
    for j = 1:length(perturb_forces)
        plot(force_results(i, j).time, force_results(i, j).F_contact, 'LineWidth', 1.5, 'DisplayName', sprintf('F = %.1f N', force_results(i, j).F_ext));
    end
    xlabel('Tiempo [s]', 'FontSize', 12);
    ylabel('Fuerza de contacto [N]', 'FontSize', 12);
    title(sprintf('D = %.1f Ns/m', D_values(i)), 'FontSize', 14);
    legend show;
    grid on;
end

% Gráfica del error final promedio vs. D
final_errors = zeros(1, length(D_values));
for i = 1:length(D_values)
    total_error = 0;
    for j = 1:length(perturb_forces)
        error = calculate_error(force_results(i, j).F_contact);
        total_error = total_error + error;
    end
    final_errors(i) = total_error / length(perturb_forces);
end

figure;
plot(D_values, final_errors, '-o', 'LineWidth', 1.5, 'MarkerSize', 8);
grid on;
xlabel('Amortiguamiento D [Ns/m]', 'FontSize', 12);
ylabel('Error final promedio [N]', 'FontSize', 12);
title('Error final promedio vs. Amortiguamiento D', 'FontSize', 14);

% Funciones auxiliares
function dx = dynamics(t, x, x_eq, Kp, Kd, Ki, l1, l2, m1, m2, g, D, F_ext)
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

    % Matriz de Coriolis
    C1 = -m2*l1*l2*sin(theta2)theta_dot2(2*theta_dot1 + theta_dot2);
    C2 = m2*l1*l2*sin(theta2)*theta_dot1^2;
    C = [C1; C2];

    % Gravedad
    G1 = (m1*l1/2 + m2*l1)*g*cos(theta1) + m2*l2*g*cos(theta1 + theta2)/2;
    G2 = m2*l2*g*cos(theta1 + theta2)/2;
    G = [G1; G2];

    % Control
    theta = [theta1; theta2];
    theta_dot = [theta_dot1; theta_dot2];
    error = theta - x_eq;
    integral_error = [integral_error1; integral_error2] + error * 0.01;
    
    tau_PD = -Kp * error - Kd * theta_dot;
    tau_I = -Ki * integral_error;
    tau_damping = -D * theta_dot;
    tau = tau_PD + tau_I + tau_damping + G + C + [F_ext; 0];

    % Dinámica del sistema
    theta_ddot = M \ tau;
    dx = [theta_dot; theta_ddot; error];
end

function F_contact = calculate_contact_force(x, l1, l2, m1, m2, g, D)
    theta1 = x(:, 1);
    theta2 = x(:, 2);
    theta_dot1 = x(:, 3);
    theta_dot2 = x(:, 4);

    % Gravedad
    G1 = (m1*l1/2 + m2*l1)*g.*cos(theta1) + m2*l2*g.*cos(theta1 + theta2)/2;
    G2 = m2*l2*g.*cos(theta1 + theta2)/2;
    G = [G1, G2];

    % Amortiguamiento
    tau_damping = -D * [theta_dot1, theta_dot2];

    % Fuerza de contacto neta
    F_contact = sqrt(sum((tau_damping + G).^2, 2));
end

function error = calculate_error(F_contact)
    error = sum(abs(F_contact - mean(F_contact))) / length(F_contact);
end
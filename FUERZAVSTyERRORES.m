% Gráfica de métricas en función de la fuerza de perturbación
forces = [results.perturb_force];
errors_acumulados = [results.error_acumulado];
tiempos_recuperacion = [results.recovery_time];

% Gráfica del error acumulado
figure;
plot(forces, errors_acumulados, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 8);
grid on;
xlabel('Fuerza de perturbación [N]', 'FontSize', 12);
ylabel('Error acumulado [m]', 'FontSize', 12);
title('Error acumulado vs. Fuerza de perturbación', 'FontSize', 14);

% Gráfica del tiempo de recuperación
figure;
plot(forces, tiempos_recuperacion, 'r-o', 'LineWidth', 1.5, 'MarkerSize', 8);
grid on;
xlabel('Fuerza de perturbación [N]', 'FontSize', 12);
ylabel('Tiempo de recuperación [s]', 'FontSize', 12);
title('Tiempo de recuperación vs. Fuerza de perturbación', 'FontSize', 14);

% Añadir anotaciones si algún tiempo es NaN
for i = 1:length(tiempos_recuperacion)
    if isnan(tiempos_recuperacion(i))
        text(forces(i), max(ylim) - 0.5, 'No converge', 'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', 'r');
    end
end
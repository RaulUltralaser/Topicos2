clc;
clearvars;
close all;

L = [1 -1 0 0 -1;
     -1 2 -1 0 0;
     0 -1 1 0 0;
     0 0 -1 1 0;
     0 0 0 0 0];

% Condiciones iniciales para las coordenadas por separado
x_x = [1; 2; 3; 4]; % Coordenadas x
x_y = [1; 2; 3; 4]; % Coordenadas y

% Inicializar el vector combinado
x = zeros(2 * length(x_x), 1);

% Combinar los vectores para la simulación
for i = 1:length(x_x)
    x(2*i-1) = x_x(i);
    x(2*i) = x_y(i);
end


% Desplazamientos relativos (deltas) entre los agentes
delta_ji = [0 2; -2 0; 0 -2; 2 0]; % [delta_1; delta_2; delta_3; delta_4]
delta = reshape(delta_ji', [], 1); % Convertir las deltas a un vector columna

% Función de referencia virtual xi^r
xi_r = @(t) [30*sin(pi*t/100); 20*sin(pi*t/50)];


% Parámetros constantes para el controlador
alpha_i = 1;
gamma_i = 1;
k_ij = 1;

% Tiempo total de simulación y período de muestreo
tiempo = 100;
Dt = 0.01;
iteraciones = tiempo / Dt;

% Ganancia proporcional
k_p = 1;

% Inicializar la matriz de estados para almacenar los resultados de la simulación
x_states = zeros(size(x, 1), iteraciones + 1);
x_states(:, 1) = x;

% Inicializar el vector de desplazamientos y referencia virtual
delta_dot = zeros(size(delta));
xi_r_dot = zeros(2, iteraciones + 1);

% Simulación con el controlador especificado
for k = 1:iteraciones
    % Calcular el término de controlador para cada agente
    u = zeros(size(x));
    eta = zeros(size(x, 1), 1);
    
    for i = 1:size(L, 1)
        for j = 1:size(L, 2)
            if L(i, j) ~= 0 % Hay una conexión entre i y j
                if j == size(L, 1) + 1 % Conexión con la referencia virtual xi^r
                    u(2*i-1:2*i) = u(2*i-1:2*i) + (k_ij * xi_r_dot(:, k) - gamma_i * (x_states(2*i-1:2*i, k) - delta(2*i-1:2*i) - xi_r(k)));
                    eta(2*i-1:2*i) = eta(2*i-1:2*i) + k_ij;
                else % Conexión entre los agentes i y j
                    u(2*i-1:2*i) = u(2*i-1:2*i) + (k_ij * (x_states(2*j-1:2*j, k) - x_states(2*i-1:2*i, k)) - gamma_i * ((x_states(2*i-1:2*i, k) - x_states(2*j-1:2*j, k)) - (delta(2*i-1:2*i) - delta(2*j-1:2*j))));
                    eta(2*i-1:2*i) = eta(2*i-1:2*i) + k_ij;
                end
            end
        end
    end
    
    % Aproximación de Euler para los estados
    x_states(:, k+1) = x_states(:, k) + Dt * u;
    
    % Actualizar las derivadas temporales de delta y xi_r
    delta_dot = (x_states(:, k+1) - x_states(:, k)) / Dt;
    xi_r_dot(:, k+1) = (xi_r(k+1) - xi_r(k)) / Dt;
end


% Vector de tiempo para graficar
t = linspace(0, tiempo, iteraciones + 1);

% Subplot para consenso en el eje x
figure;
subplot(2, 1, 1);
hold on;
for i = 1:2:size(x_states, 1) % Consenso en x
     plot(t, x_states(i, :));
end
hold off;
xlabel('Tiempo (s)');
ylabel('\xi_{x}');
title('Consenso en el eje x');
grid on;

% Subplot para consenso en el eje y
subplot(2, 1, 2);
hold on;
for i = 2:2:size(x_states, 1) % Consenso en y
    plot(t, x_states(i, :));
end
hold off;
xlabel('Tiempo (s)');
ylabel('\xi_{y}');
title('Consenso en el eje y');
grid on;

% Graficar trayectorias en el plano x-y
figure;
hold on;
colors = ['r', 'g', 'b', 'm']; % Colores diferentes para cada agente
for i = 1:length(x_x)
    plot(x_states(2*i-1, :), x_states(2*i, :), 'Color', colors(i), 'DisplayName', ['Agente ' num2str(i)]);
end
hold off;
xlabel('Posición en x');
ylabel('Posición en y');
title('Trayectorias de los agentes en el plano x-y');
legend show;
grid on;

% Graficar la referencia virtual xi^r
figure;
plot(t, xi_r(1:end-1), 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('\xi^r');
title('Referencia Virtual \xi^r(t)');
grid on;

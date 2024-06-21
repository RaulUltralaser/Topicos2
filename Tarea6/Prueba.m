clc;
clearvars;
close all;

L = [1 -1 0 0 -1;
    -1 2 -1 0 0;
     0 -1 1 0 0;
     0 0 -1 1 0;
     0 0  0 0 0];

% Condiciones iniciales para las coordenadas por separado
x_x = [0; 0; 0; 0]; % Coordenadas x
x_y = [0; 0; 0; 4]; % Coordenadas y

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

% Generar el archivo de tiempo
tiempo = 100;
t = linspace(0, tiempo, 1000);

% Función de referencia virtual xi^r
xi_r = [30*sin(pi*t/100); 20*sin(pi*t/50)];

% Derivada de la referencia
f = [30*(pi/100)*cos(pi*t/100); 20*(pi/50)*cos(pi*t/50)];

% Parámetros constantes para el controlador
alpha_i = 1;
gamma_i = 1;
k_ij = 1;

% Tiempo total de simulación y período de muestreo
Dt = 0.01;
iteraciones = tiempo / Dt;

% Ganancia proporcional
k_p = 1;

% Inicializar la matriz de estados para almacenar los resultados de la simulación
xi = zeros(size(x, 1), iteraciones + 1);
xi(:, 1) = x;

% Inicializar el vector de desplazamientos y referencia virtual
delta_dot = zeros(size(delta));

% Simulación con el controlador especificado
for k = 1:iteraciones
    % Calcular el término de controlador para cada agente
    u = zeros(size(2*x));
    eta = zeros(size(L, 1), 1);
    
    for i = 1:size(L, 1)
        for j = 1:size(L, 2)
            if L(i, j) ~= 0 % Hay una conexión entre i y j
                g_ij = 1;
            else
                g_ij = 0;
            end
            if L(i, end) ~= 0 % Hay una conexión entre la referencia y el vehículo
                g_in1 = 1;
            else
                g_in1 = 0;
            end
            eta(i) = g_in1 * alpha_i + g_ij * k_ij;
            % Control en las coordenadas x y y
            u(2*i-1:2*i) = delta_dot(2*i-1:2*i) + ...
                (1 / eta(i)) * (g_ij * k_ij * (xi(2*j-1:2*j, k) - delta_dot(2*j-1:2*j) - gamma_i * (xi(2*i-1:2*i, k) - xi(2*j-1:2*j, k) - (delta(2*i-1:2*i) - delta(2*j-1:2*j))))) + ...
                (1 / eta(i)) * g_in1 * alpha_i * (f(:, k) - gamma_i * (xi(2*i-1:2*i, k) - delta(2*i-1:2*i) - xi_r(:, k)));
        end
    end
    
    % Aproximación de Euler para los estados
    xi(:, k+1) = xi(:, k) + Dt * u;
end

% Graficar la trayectoria de los agentes
figure;
hold on;
for i = 1:size(L, 1)
    plot(xi(2*i-1, :), xi(2*i, :), 'DisplayName', ['Agente ', num2str(i)]);
end
xlabel('X');
ylabel('Y');
title('Trayectoria de los agentes');
legend;
grid on;
hold off;

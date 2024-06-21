clc;
clearvars;
close all;

% Matriz Laplaciana (los pesos son 1, así que usamos directamente la matriz de adyacencia)
L = [1 -1 0 0;
     -1 2 -1 0;
     0 -1 1 0;
     0 0 -1 1];

% Matriz identidad de tamaño 2
I = eye(2);

% Condiciones iniciales para las coordenadas por separado
x_x = [0; 0; 0; 0]; % Coordenadas x
x_y = [0; 0; 0; 0]; % Coordenadas y

% Inicializar el vector combinado
x = zeros(2 * length(x_x), 1);

% Combinar los vectores para la simulación
for i = 1:length(x_x)
    x(2*i-1) = x_x(i);
    x(2*i) = x_y(i);
end
x0=x;



% Desplazamientos deseados (deltas)
delta_ji = [0 2; -2 0; 0 -2; 2 0]; % [delta_1; delta_2; delta_3; delta_4]
delta = reshape(delta_ji', [], 1); % Convertir las deltas a un vector columna

% Parámetros de simulación
Dt = 0.01; % Periodo de muestreo
tiempo = 10; % Tiempo total en segundos
iteraciones = tiempo / Dt;

% Ganancia proporcional
k_p = 10;

% Inicializar la matriz de estados para almacenar los resultados de la simulación
x_forma = zeros(size(x, 1), iteraciones + 1);
x_forma(:, 1) = x;

% Simulación con control de formación basado en \delta_{ji}^*
for k = 1:iteraciones
    u = zeros(size(x)); % Inicializar la entrada de control
    for i = 1:size(L, 1)
        for j = 1:size(L, 2)
            if L(i, j) ~= 0 % Verificar si hay una conexión
                % Aplicar control de formación con delta_{ji}^*
                u(2*i-1:2*i) = u(2*i-1:2*i) + k_p * (x_forma(2*j-1:2*j, k) - x_forma(2*i-1:2*i, k) - delta(2*i-1:2*i) + delta(2*j-1:2*j));
            end
        end
    end
    % Aproximación de Euler
    x_forma(:, k+1) = x_forma(:, k) + Dt * u;
end

% Vector de tiempo para graficar
t = linspace(0, tiempo, iteraciones + 1);
P=zeros(2,1);
% Graficar las posiciones de los agentes en el plano xy con sus posiciones finales
figure;
hold on;
num_agents = size(x0, 1) / 2;
colors = lines(num_agents);
for i = 1:num_agents
    plot(x_forma(2*i-1, :) + P(1), x_forma(2*i, :) + P(2), 'DisplayName', ['Agente ' num2str(i)], 'Color', colors(i, :), 'LineStyle', '--');
    plot(x_forma(2*i-1, end) + P(1), x_forma(2*i, end) + P(2), 'o', 'Color', colors(i, :), 'MarkerFaceColor', colors(i, :));
end

% Agregar líneas entre los agentes especificados
plot([x_forma(1,end) + P(1) x_forma(7,end) + P(1)], [x_forma(2,end) + P(2) x_forma(8,end) + P(2)], 'k-'); % Línea entre Agente 1 y Agente 4
plot([x_forma(1,end) + P(1) x_forma(3,end) + P(1)], [x_forma(2,end) + P(2) x_forma(4,end) + P(2)], 'k-'); % Línea entre Agente 1 y Agente 2
plot([x_forma(7,end) + P(1) x_forma(5,end) + P(1)], [x_forma(8,end) + P(2) x_forma(6,end) + P(2)], 'k-'); % Línea entre Agente 4 y Agente 3
plot([x_forma(3,end) + P(1) x_forma(5,end) + P(1)], [x_forma(4,end) + P(2) x_forma(6,end) + P(2)], 'k-'); % Línea entre Agente 2 y Agente 3

hold off;
xlabel('Posición en x');
ylabel('Posición en y');
title('Trayectorias de los agentes en el plano xy');
axis equal;
grid on;
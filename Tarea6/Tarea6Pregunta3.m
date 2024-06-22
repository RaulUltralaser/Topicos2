clear all 
clc 
close all

A = [ 0  1  0  0 
      1  0  1  0 
      0  1  0  0 
      0  0  1  0 ];

Lg_b = [ 1 -1  0  0 
        -1  2 -1  0 
         0 -1  1  0 
         0  0 -1  1 ];
eig(Lg_b)
I = eye(2);
I2 = zeros(8,8);
I2(1,1) = 1;
I2(2,2) = 1;
I3 = eye(8,8);
I3(1,1) = 1/2;
I3(2,2) = 1/2;
I3(3,3) = 1/2;
I3(4,4) = 1/2;

% Ganancias
gamma = 1;
g = 1;
alpha = 1;

% Condiciones iniciales de los agentes en x e y 
x_init = [ 0; -4;  0;  4 ];  % Condiciones iniciales en x
y_init = [ 2;  0; -4;  0 ];  % Condiciones iniciales en y

% Número de agentes
num_agents = length(x_init);

% Combinar condiciones iniciales en un solo vector
x0 = zeros(2 * num_agents, 1);
for i = 1:num_agents
    x0(2*i - 1) = x_init(i);
    x0(2*i) = y_init(i);
end

% Tiempo de simulación
t0 = 0; % tiempo inicial
tf = 30; % tiempo final
h = 0.001; % paso de tiempo

% Inicializar las variables
t = t0:h:tf;
n = length(t);

% Desplazamientos deseados en x e y 
d_x = [ 0; -2;  0;  2 ];  % Desplazamientos deseados en x
d_y = [ 2;  0; -2;  0 ];  % Desplazamientos deseados en y

% Inicializar los desplazamientos deseados y sus derivadas
d = zeros(2 * num_agents, 1);

% Valores iniciales para los desplazamientos deseados
for i = 1:num_agents
    d(2*i - 1) = d_x(i);
    d(2*i) = d_y(i);
end

x = x0;

% Inicializar la matriz para almacenar el error relativo
error_relativo = zeros(1, n+1 );

% Calcular el error inicial
error_relativo(1) = norm(kron(Lg_b, I) * (x0 - d));

% Inicializar a la referencia y su derivada
ref = zeros(8,1);
dref = zeros(8,1);
dref(1) = 30 * (pi/100);
dref(2) = 20 * (pi/50);

x_dot = zeros(8,1);

% Tiempos específicos para almacenar las posiciones
t_intervals = [0, 10, 15, 20];
interval_indices = round(t_intervals / h) + 1; % Convertir tiempos a índices

% Inicializar las posiciones en tiempos específicos
positions_at_intervals = cell(length(t_intervals), 1);

for i = 1:n
    % Cálculo de la dinámica del sistema con el desplazamiento relativo
    x(:, i + 1) = x(:, i) + h * (I3 * alpha * I2 * (dref - gamma * (x(:, i) - d - ref)) + g * I3 * (kron(A, I) * x_dot - gamma * (kron(Lg_b, I) * (x(:, i) - d))));
    x_dot = (I3 * alpha * I2 * (dref - gamma * (x(:, i) - d - ref)) + g * I3 * (kron(A, I) * x_dot - gamma * (kron(Lg_b, I) * (x(:, i) - d))));
    
    % Calcular el error en cada iteración
    error_relativo(i + 1) = norm(kron(Lg_b, I) * (x(:, i + 1) - d));
    
    % Dinámica de la referencia
    ref(1) = 30 * sin(pi * t(i) / 100);
    ref(2) = 20 * sin(pi * t(i) / 50);
    
    % Guardar la referencia en la trayectoria
    ref_traj(1, i+1) = ref(1);
    ref_traj(2, i+1) = ref(2);
    
    % Derivadas de la referencia
    dref(1) = 30 * (pi / 100) * cos(pi * t(i) / 100); 
    dref(2) = 20 * (pi / 50) * cos(pi * t(i) / 50);
    
    % Almacenar posiciones en tiempos específicos
    if ismember(i, interval_indices)
        positions_at_intervals{find(interval_indices == i)} = x(:, i);
    end
end

% Graficar el error relativo respecto al tiempo
t = linspace(t0, tf, n+1);
figure;
plot(t, error_relativo);
xlabel('Tiempo (s)');
ylabel('Error relativo');
title('Error relativo de la formación respecto al tiempo');
grid on;

% Graficar las posiciones de los agentes en el plano xy con sus posiciones finales
figure;
hold on;
num_agents = size(x0, 1) / 2;
colors = lines(num_agents);

% Graficar trayectorias
for i = 1:num_agents
    plot(x(2*i-1, :), x(2*i, :), 'DisplayName', ['Agente ' num2str(i)], 'Color', colors(i, :), 'LineStyle', '--');
    plot(x(2*i-1, end), x(2*i, end), 'o', 'Color', colors(i, :), 'MarkerFaceColor', colors(i, :));
end

% Graficar las posiciones a los tiempos específicos
markers = {'s', 'd', '^', 'v'}; % Marcadores para tiempos específicos
for j = 1:length(t_intervals)
    pos = positions_at_intervals{j};
    for i = 1:num_agents
        plot(pos(2*i-1), pos(2*i), markers{j}, 'Color', colors(i, :), 'MarkerFaceColor', colors(i, :), 'DisplayName', ['Agente ' num2str(i) ' @ ' num2str(t_intervals(j)) 's']);
    end
    
    % Agregar líneas entre los agentes en tiempos específicos
    plot([pos(1) pos(7)], [pos(2) pos(8)], 'k-'); % Línea entre Agente 1 y Agente 4
    plot([pos(1) pos(3)], [pos(2) pos(4)], 'k-'); % Línea entre Agente 1 y Agente 2
    plot([pos(7) pos(5)], [pos(8) pos(6)], 'k-'); % Línea entre Agente 4 y Agente 3
    plot([pos(3) pos(5)], [pos(4) pos(6)], 'k-'); % Línea entre Agente 2 y Agente 3
end

% Agregar líneas entre los agentes especificados en la posición final
plot([x(1,end) x(7,end)], [x(2,end) x(8,end)], 'k-'); % Línea entre Agente 1 y Agente 4
plot([x(1,end) x(3,end)], [x(2,end) x(4,end)], 'k-'); % Línea entre Agente 1 y Agente 2
plot([x(7,end) x(5,end)], [x(8,end) x(6,end)], 'k-'); % Línea entre Agente 4 y Agente 3
plot([x(3,end) x(5,end)], [x(4,end) x(6,end)], 'k-'); % Línea entre Agente 2 y Agente 3

% Graficar la trayectoria de la referencia
plot(ref_traj(1, :), ref_traj(2, :), 'r-', 'DisplayName', 'Referencia', 'LineWidth', 1.5);

hold off;
xlabel('Posición en x');
ylabel('Posición en y');
title('Trayectorias de los agentes en el plano xy');
legend show;
axis equal;
grid on;

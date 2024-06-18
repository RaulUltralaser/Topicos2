% Parámetros de la curva de Lissajous
A = 2;          % Amplitud en x
B = 1;          % Amplitud en y
a = 1;          % Frecuencia en x
b = 2;          % Frecuencia en y
delta = 0;      % Diferencia de fase

% Tiempo
t = linspace(0, 10, 1000); % Aumentar el rango de tiempo para la simulación

% Ecuaciones paramétricas de la curva de Lissajous
x_curve = A * sin(a * t + delta);
y_curve = B * sin(b * t);

% Derivadas
dx_dt = A * a * cos(a * t + delta);          % Primera derivada de x(t)
dy_dt = B * b * cos(b * t);                  % Primera derivada de y(t)

% Distancia desde la curva para los agentes
d = 1;

% Posiciones relativas de los agentes con respecto a la curva
offsets = [d, 0; 0, d; -d, 0; 0, -d];

% Parámetros de control
k_a = 1;
k_v = 2;
k_p = 5;

% Inicializar el vector de estados p para los agentes
num_agents = 4;
states = zeros(num_agents, 4, length(t)); % [posición_x, velocidad_x, posición_y, velocidad_y] para cada agente en cada tiempo

% Inicializar estados
for i = 1:num_agents
    states(i, 1, 1) = x_curve(1) + offsets(i, 1); % Posiciones iniciales en x
    states(i, 3, 1) = y_curve(1) + offsets(i, 2); % Posiciones iniciales en y
end

% Simulación
dt = t(2) - t(1);
AA = [0, 1, 0, 0; 0, 0, 0, 0; 0, 0, 0, 1; 0, 0, 0, 0];
BB = [0, 0; 1, 0; 0, 0; 0, 1];

for k = 1:length(t)-1
    % Posiciones deseadas en el siguiente instante de tiempo
    x_des = x_curve(k+1);
    y_des = y_curve(k+1);
    
    for i = 1:num_agents
        p_des = [x_des + offsets(i, 1); y_des + offsets(i, 2)];
        v_des = [dx_dt(k+1); dy_dt(k+1)];

        % Posicion actual
        p_real = states(i, [1, 3], k);
        v_real = states(i, [2, 4], k);

        % Error de posicion
        e_p = p_des - p_real';

        % Error de velocidad
        e_v = v_des - v_real';

        % Control de aceleracion
        u = -k_a * (k_v * e_v - k_p * e_p);

        % Actualizar estados
        states(i, 2, k+1) = states(i, 2, k) + u(1) * dt; % velocidad_x
        states(i, 4, k+1) = states(i, 4, k) + u(2) * dt; % velocidad_y
        states(i, 1, k+1) = states(i, 1, k) + states(i, 2, k) * dt; % posición_x
        states(i, 3, k+1) = states(i, 3, k) + states(i, 4, k) * dt; % posición_y
    end
end

% Graficar la curva de Lissajous y las trayectorias de los agentes
figure;
hold on;

% Graficar la curva de Lissajous
plot(x_curve, y_curve, 'k', 'LineWidth', 1.5);

% Colores para los agentes
colors = ['r', 'g', 'b', 'm'];

% Graficar las trayectorias de los agentes
for i = 1:num_agents
    plot(squeeze(states(i, 1, :)), squeeze(states(i, 3, :)), 'Color', colors(i), 'LineWidth', 1.5);
    plot(states(i, 1, end), states(i, 3, end), 'o', 'Color', colors(i), 'MarkerSize', 10, 'LineWidth', 2); % Posición final
end

title('Curva de Lissajous con trayectorias de agentes controlados');
xlabel('x(t)');
ylabel('y(t)');
grid on;
axis equal;
legend('Curva de Lissajous', 'Agente 1', 'Agente 2', 'Agente 3', 'Agente 4');
hold off;

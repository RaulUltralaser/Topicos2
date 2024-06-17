clc
clearvars 
close all 

% Parámetros de la curva de Lissajous
A = 2;          % Amplitud en x
B = 1;          % Amplitud en y
a = 1;          % Frecuencia en x
b = 2;          % Frecuencia en y
delta = 0;      % Diferencia de fase

AA=[0 1 0 0;
    0 0 0 0;
    0 0 0 1;
    0 0 0 0];

BB=[0 0;
    1 0;
    0 0;
    0 1];

% Tiempo
t = linspace(0, 20, 1000); % Aumentar el rango de tiempo para la simulación

% Ecuaciones paramétricas de la curva de Lissajous
x_curve = A * sin(a * t + delta);
y_curve = B * sin(b * t);

% Distancia desde la curva para los agentes
d = 1;

% Posiciones relativas de los agentes con respecto a la curva
x_offset = [d, 0, -d, 0];
y_offset = [0, -d, 0, d];

% Parámetros de control
k_a = 1;
k_v = 10;
k_p = 10;

% Inicializar posiciones y velocidades de los agentes
num_agents = 4;
positions = zeros(num_agents, 2, length(t)); % [x, y] para cada agente en cada tiempo
velocities = zeros(num_agents, 2, length(t)); % [vx, vy] para cada agente en cada tiempo

% Inicializar estados
%Primer punto
positions(:, 1, 1) = x_curve(1) + x_offset; % Posiciones iniciales en x
positions(:, 2, 1) = y_curve(1) + y_offset; % Posiciones iniciales en y
%Segundo punto
% positions(:, 1, 1) = x_curve(1) + [rand(),rand(),rand(),rand()]; % Posiciones iniciales en x
% positions(:, 2, 1) = y_curve(1) + [rand(),rand(),rand(),rand()]; %Posiciones iniciales en y
% Simulación
dt = t(2) - t(1);
for k = 1:length(t)-1
    % Posiciones deseadas en el siguiente instante de tiempo
    x_des = x_curve(k+1) + x_offset;
    y_des = y_curve(k+1) + y_offset;

    for i = 1:num_agents
        % Error de posición y velocidad
        e_p = [x_des(i) - positions(i, 1, k); y_des(i) - positions(i, 2, k)];
        e_v = -[velocities(i, 1, k); velocities(i, 2, k)];

        % Control de aceleración
        u = -k_a * (k_v * e_v + k_p * e_p);

        % Actualizar posiciones
        velocities(i, 1, k+1) = velocities(i, 1, k) - u(1) * dt;
        velocities(i, 2, k+1) = velocities(i, 2, k) - u(2) * dt;
        positions(i, 1, k+1) = positions(i, 1, k) + velocities(i, 1, k) * dt;
        positions(i, 2, k+1) = positions(i, 2, k) + velocities(i, 2, k) * dt;
        %Guardo estos datos para ploteos
        epx(k)=e_p(1); %Error posición en x
        epy(k)=e_p(2); %Error posición en y

        evx(k)=e_v(1); %Error velocidad en x
        evy(k)=e_v(2); %Error velocidad en y

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
    plot(squeeze(positions(i, 1, :)), squeeze(positions(i, 2, :)), 'Color', colors(i), 'LineWidth', 1.5);
    plot(positions(i, 1, 1), positions(i, 2, 1), 'o', 'Color', colors(i), 'MarkerSize', 10, 'LineWidth', 2); % Posición final
end

title('Curva de Lissajous con trayectorias de agentes controlados');
xlabel('x(t)');
ylabel('y(t)');
grid on;
axis equal;
% legend('Curva de Lissajous', 'Agente 1', 'Agente 2', 'Agente 3', 'Agente 4');
hold off;

% figure
% subplot(1,2,1)
% plot(t(1,1:size(t,2)-1),epx);
% title('Errores de posición en x')
% ylabel('Error')
% xlabel('tiempo s')
% grid on
% subplot(1,2,2)
% plot(t(1,1:size(t,2)-1),epy);
% title('Errores de posición en y')
% ylabel('Error')
% xlabel('tiempo s')
% grid on
% 
% 
% figure
% subplot(1,2,1)
% plot(t(1,1:size(t,2)-1),evx);
% title('Errores de posición en x')
% ylabel('Error')
% xlabel('tiempo s')
% grid on
% subplot(1,2,2)
% plot(t(1,1:size(t,2)-1),evy);
% title('Errores de posición en y')
% ylabel('Error')
% xlabel('tiempo s')
% grid on


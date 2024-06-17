clc
clearvars 
close all 

% Parámetros de la curva de Lissajous
A = 2;          % Amplitud en x
B = 1;          % Amplitud en y
a = 1;          % Frecuencia en x
b = 2;          % Frecuencia en y
delta = 0;      % Diferencia de fase

AA=[0 0 1 0;
    0 0 0 1;
    0 0 0 0;
    0 0 0 0];

BB=[0 0;
    0 0;
    1 0;
    0 1];

% Tiempo
t = linspace(0, 10, 1000); % Aumentar el rango de tiempo para la simulación

% Ecuaciones paramétricas de la curva de Lissajous
x_curve = A * sin(a * t + delta);
y_curve = B * sin(b * t);

vx_curve= A*a*cos(a*t);
vy_curve= B*b*cos(b*t);

% Distancia desde la curva para los agentes
d = 1;

% Posiciones relativas de los agentes con respecto a la curva
x_offset = [d, 0, -d, 0];
y_offset = [0, -d, 0, d];

% Parámetros de control
k_a = 1;
k_v = 5;
k_p = 10;

% Inicializar posiciones y velocidades de los agentes
p = zeros(4, 1); 

% Inicializar estados
%Primer punto
p(:,1)=[x_curve(1) + x_offset(1);y_curve(1) + y_offset(1);0;0];

% Simulacion
dt = t(2) - t(1);
for k = 1:length(t)-1
    % Posiciones deseadas en el siguiente instante de tiempo
    p_des = [x_curve(k+1) + 1;y_curve(k+1)];
    % Posicion actual
    p_real= p(1:2,k);
    %Error de posicion
    e_p = p_des-p_real;

    %Velocidad deseada
    v_des=[vx_curve(k+1);vy_curve(k+1)];
    %Velocidad actual
    v_real=p(3:4,k);
    %Error de velocidad
    e_v = v_des-v_real;
%     e_v=-k_p*e_p;

    % Control de aceleracion
    u = -k_a * (k_v * e_v - k_p * e_p);

    %Sistema
    p(:,k+1)=p(:,k)-(AA*p(:,k)+BB*u)*dt;

  
end

% Graficar la curva de Lissajous y las trayectorias de los agentes
figure;
hold on;

% Graficar la curva de Lissajous
plot(x_curve, y_curve, 'k', 'LineWidth', 1.5);

% Colores para los agentes
colors = ['r', 'g', 'b', 'm'];

plot(p(1, :), p(2, :), 'Color', colors(1), 'LineWidth', 1.5);


title('Curva de Lissajous con trayectorias de agentes controlados');
xlabel('x(t)');
ylabel('y(t)');
grid on;
axis equal;
hold off;


% Graficar las trayectorias de los agentes
% for i = 1:num_agents
%     plot(squeeze(positions(i, 1, :)), squeeze(positions(i, 2, :)), 'Color', colors(i), 'LineWidth', 1.5);
%     plot(positions(i, 1, 1), positions(i, 2, 1), 'o', 'Color', colors(i), 'MarkerSize', 10, 'LineWidth', 2); % Posición final
% end

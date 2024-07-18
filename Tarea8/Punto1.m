clc
clearvars
close all

% Parámetros iniciales
l = 0.5; % Distancia del centro del robot al punto de control
K = 1*eye(8); % Ganancia del controlador

% Inicializar condiciones iniciales para 4 robots
z = [2; 1; 1.95; .95; 3; -1; 2; -2]; % Cada columna es [x; y] de un robot
Angulo = [90; 0; -45; -90]; % Ángulos dados en grados

% Matriz Laplaciana de un grafo dirigido en ciclo para 4 robots
L = [ 1  0  0 -1;
     -1  1  0  0;
      0 -1  1  0;
      0  0 -1  1];
%Laplaciano grafo no dirigido
% L=[2 -1  0 -1;
%   -1  2 -1  0;
%    0 -1  2 -1;
%   -1  0 -1  2];
I=eye(2);

% Datos de la simulación
Dt = 0.01; % Periodo de muestreo
tiempo = 30; % Duración de la simulación en segundos
iteraciones = tiempo / Dt;

% Inicializar trayectorias
z_hist = zeros(8, iteraciones+1);
z_hist(:, 1) = z;

theta = zeros(4, iteraciones+1);
theta(:, 1) = deg2rad(Angulo);

v_hist = zeros(4, iteraciones);
w_hist = zeros(4, iteraciones);

evx_hist = zeros(4,iteraciones);
evy_hist = zeros(4,iteraciones);

% Simulación
for k = 1:iteraciones
    % Calcular la matriz M para cada robot en cada iteración
    M1 = [cos(theta(1,k)) -l*sin(theta(1,k));
         sin(theta(1,k)) l*cos(theta(1,k))];
    M2 = [cos(theta(2,k)) -l*sin(theta(2,k));
         sin(theta(2,k)) l*cos(theta(2,k))];
    M3 = [cos(theta(3,k)) -l*sin(theta(3,k));
         sin(theta(3,k)) l*cos(theta(3,k))];
    M4 = [cos(theta(4,k)) -l*sin(theta(4,k));
         sin(theta(4,k)) l*cos(theta(4,k))];
    M = blkdiag(M1, M2, M3, M4);

    ev = -kron(L,I)*z(:,k);

    u = M \ (K * ev);
    
    % Separo la ley de control en lineal y rotacional
    v = [u(1),u(3),u(5),u(7)];
    w = [u(2),u(4),u(6),u(8)];
    
    % Dinámica del sistema
    z(:,k+1) = z(:,k) + Dt * (M * u);
    theta(:,k+1) = theta(:,k) + Dt * (w'); % Actualización del ángulo

    % Guardar el historial de valores para graficar
    z_hist(:, k+1) = z(:,k+1);
    v_hist(:,k) = v';
    w_hist(:,k) = w';
    evx_hist(:,k) = [ev(1); ev(3); ev(5); ev(7)];
    evy_hist(:,k) = [ev(2); ev(4); ev(6); ev(8)];
end

t = linspace(0, tiempo, iteraciones);

% Gráfica de resultados
figure;
hold on
plot(z_hist(1, :), z_hist(2, :),'r');
quiver(z_hist(1, 1), z_hist(2, 1), cos(theta(1,1)), sin(theta(1,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(z_hist(1, end), z_hist(2, end), cos(theta(1,end)), sin(theta(1,end)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 2);
plot(z_hist(3, :), z_hist(4, :),'b');
quiver(z_hist(3, 1), z_hist(4, 1), cos(theta(2,1)), sin(theta(2,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(z_hist(3, end), z_hist(4, end), cos(theta(2,end)), sin(theta(2,end)), 0.1, 'b', 'LineWidth', .1, 'MaxHeadSize', 2);
plot(z_hist(5, :), z_hist(6, :),'g');
quiver(z_hist(5, 1), z_hist(6, 1), cos(theta(3,1)), sin(theta(3,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(z_hist(5, end), z_hist(6, end), cos(theta(3,end)), sin(theta(3,end)), 0.1, 'g', 'LineWidth', .1, 'MaxHeadSize', 2);
plot(z_hist(7, :), z_hist(8, :),'k');
quiver(z_hist(7, 1), z_hist(8, 1), cos(theta(4,1)), sin(theta(4,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(z_hist(7, end), z_hist(8, end), cos(theta(4,end)), sin(theta(4,end)), 0.1, 'k', 'LineWidth', .1, 'MaxHeadSize', 2);
xlabel('x');
ylabel('y');
title('Trayectoria del robot no holónomo');
hold off
grid on;

% figure
% subplot(1,2,1)
% plot(t,evx_hist)
% xlabel('t');
% ylabel('error');
% title('Error en el eje x');
% grid on
% subplot(1,2,2)
% plot(t,evy_hist)
% xlabel('t');
% ylabel('error');
% title('Error en el eje x');
% grid on
% 
% figure
% hold on
% subplot(1,2,1)
% plot(t,v_hist)
% xlabel('t');
% ylabel('v');
% title('Ley de control v');
% grid on
% subplot(1,2,2)
% plot(t,w_hist)
% xlabel('t');
% ylabel('w');
% title('Ley de control w');
% grid on
% hold off
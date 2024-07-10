clc
clearvars
close all

% Parámetros iniciales
l = 1; % Distancia del centro del robot al punto de control
z = [2; 2]; % Condiciones iniciales
theta = 90; % Ángulo inicial (puede ser en grados)
K = .5; % Ganancia del controlador (ajusta según sea necesario)

% Datos de la simulación
Dt = 0.01; % Periodo de muestreo
tiempo = 15; % Duración de la simulación en segundos
iteraciones = tiempo / Dt;

% Inicializar trayectorias
z_hist = zeros(2, iteraciones+1);
z_hist(:, 1) = z;

% Inicializar trayectorias
v_hist = zeros(1,iteraciones);
% Inicializar trayectorias
w_hist = zeros(1,iteraciones);
% Simulación
for k = 1:iteraciones
    % Calcular la matriz M en cada iteración
    M = [cos(theta(k)) -l*sin(theta(k));
         sin(theta(k)) l*cos(theta(k))];
     
    % Controlador
    u = M \ (-K * z); % u = M^{-1}(-kz)
    
    % Actualizar el estado del sistema
    v = u(1);
    w = u(2);
    
    % Dinámica del sistema
    z(:,k+1) = z(:,k)-Dt*(M*z(:,k));
    theta(k+1) = theta(k)-Dt*(w*theta(k)) ; % Actualización del ángulo

    %Errores
    ex(k)=z(1,k);
    ey(k)=z(2,k);
    
    % Guardar el historial de posiciones
    z_hist(:, k+1) = z(:,k+1);
    v_hist(k)=v;
    w_hist(k)=w;
end

t=linspace(0,tiempo,iteraciones);

% Gráfica de resultados
figure;
plot(z_hist(1, :), z_hist(2, :));
xlabel('x');
ylabel('y');
title('Trayectoria del robot no holónimo');
grid on;

figure
subplot(1,2,1)
plot(t,ex)
xlabel('t');
ylabel('error');
title('Error en el eje x');
grid on
subplot(1,2,2)
plot(t,ey)
plot(t,ex)
xlabel('t');
ylabel('error');
title('Error en el eje y');
grid on


figure
subplot(1,2,1)
plot(t,v_hist)
xlabel('t');
ylabel('v');
title('Ley de control v');
grid on
subplot(1,2,2)
plot(t,w_hist)
xlabel('t');
ylabel('w');
title('Ley de control w');
grid on

clc
close all
clearvars 

% Coordenadas deseadas
p1 = [0; 0];
p2 = [1; 0];
p3 = [1; -1];
p4 = [0; -1];

% Definir las diferencias cuadradas normadas d^2
d2 =[(p1 - p2)'*(p1 - p2);
      (p1 - p4)'*(p1 - p4);
      (p2 - p3)'*(p2 - p3);
      (p4 - p3)'*(p4 - p3);
      (p1 - p3)'*(p1 - p3)];

%producto kronecker
I=eye(2);

% Periodo de muestreo
Dt = 0.01;
tiempo = 15; % segundos
iteraciones = tiempo / Dt;

% Posiciones iniciales de los nodos
pInit = [0;0;1;0;1;-1;0;-1];

% Inicializo el valor de p
p = zeros(8, iteraciones); 
p(:, 1) = pInit;
sigma = zeros(5, iteraciones);

% Parámetros para la trayectoria circular del agente 1
radio = 0.5;
frecuencia = 2 * pi / tiempo;

% Simulación con aproximación de Euler
for k = 1:iteraciones
    % Trayectoria circular para el agente 1
    p(1:2, k) =  radio * [cos(frecuencia * k * Dt); sin(frecuencia * k * Dt)]-radio*[cos(frecuencia  * Dt);sin(frecuencia  * Dt)];
    %% MINIMAMENTE RIGIDA
    e1 = p(3:4, k) - p(1:2, k); % e1 = p2 - p1
    e2 = p(7:8, k) - p(1:2, k); % e2 = p4 - p1
    e3 = p(5:6, k) - p(3:4, k); % e3 = p3 - p2
    e4 = p(5:6, k) - p(7:8, k); % e4 = p3 - p4
    e5 = p(5:6, k) - p(1:2, k); % e5 = p3 - p1
    e_2 = blkdiag(e1', e2', e3', e4', e5');
    
    % Calculo los valores de sigma
    sigma1 = e1' * e1 - d2(1);
    sigma2 = e2' * e2 - d2(2);
    sigma3 = e3' * e3 - d2(3);
    sigma4 = e4' * e4 - d2(4);
    sigma5 = e5' * e5 - d2(5);
    sigma(:, k) = [sigma1; sigma2; sigma3; sigma4; sigma5];
    
    % Matrices de incidencia locales cosa rigida
    E = [1 -1  0  0;
         1  0  0 -1;
         0  1  0 -1;
         0  0 -1  1;
         1  0 -1  0];
    kronp = kron(E, I);
    
    % Calculo del vector R
    R = e_2 * kronp;
    
    % Calcular la derivada de p para los nodos controlados
    if k < iteraciones
        u = -R' * R * p(:, k) - R' * d2;
        % Aproximación de Euler
        p(3:end, k+1) = p(3:end, k) + Dt * u(3:end);
    end
end

figure
hold on
plot(p(1,:), p(2,:))
plot(p(3,:), p(4,:))
plot(p(5,:), p(6,:))
plot(p(7,:), p(8,:))
% Lineas entre nodos
plot([p(1, end), p(3, end)], [p(2, end), p(4, end)], 'k--');
plot([p(1, end), p(7, end)], [p(2, end), p(8, end)], 'k--');
plot([p(3, end), p(5, end)], [p(4, end), p(6, end)], 'k--');
plot([p(7, end), p(5, end)], [p(8, end), p(6, end)], 'k--');
plot([p(1, end), p(5, end)], [p(2, end), p(6, end)], 'k--');
title('Evolución de posiciones con un agente en trayectoria circular')
xlabel('x')
ylabel('y')
axis equal
grid on
hold off

t = linspace(0, tiempo, iteraciones);
figure
plot(t,sigma)
grid on

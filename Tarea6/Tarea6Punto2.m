clc
clearvars 
close all 


q=[1/3 1/3 1/3 0];

L=[1 -1  0 0;
  -1  2 -1 0;
   0 -1  1 0;
   0  0 -1 1];

[V,D]=eig(L);

%La matriz diagonal de unos
I=eye(2);

%Producto Kronecker
KL=kron(L,I);

% Definir las condiciones inciales para las coordenadas por separado
x_x = [8; 4; 2; 0]; %Esto es en x
x_y = [5; 3; 10; 1]; %Esto es en y

% Inicializar el vector
x = zeros(2 * length(x_x), 1);

% Combinarlos para la simlacion
for i = 1:length(x_x)
    x(2*i-1) = x_x(i);
    x(2*i) = x_y(i);
end
x0=x;

% Formación deseada (especificada relativamente)
 delta=[ 1; % d21 x
         0; % d21 y
        -1; % d12 - d32 x
        -3; % d12 - d32 y
         0; % d23 x
         3; % d23 y
         1; % d34 x
         0]; % d34 y
P=zeros(2,1);


%Datos de la simulacion
%periodo de muestreo
Dt=0.01;
tiempo=10; %segundos
iteraciones=tiempo/Dt;

% Inicializar la matriz de estados
x_forma = zeros(size(x, 1), iteraciones + 1);
x_forma(:, 1) = x0;

% Inicializar la matriz para almacenar el error relativo
error_relativo = zeros(1, iteraciones + 1);

% Calcular el error inicial
error_relativo(1) = norm(KL * x0 + delta);

% Simulacion
for k = 1:iteraciones
    %Aproximación de Euler con Laplaciano
    x_forma(:, k+1) = x_forma(:, k) - Dt * (KL * x_forma(:, k) + delta);
    % Calcular el error en cada iteración
    error_relativo(k + 1) = norm(KL * x_forma(:, k + 1) + delta);
end

% Graficar el error relativo respecto al tiempo
t = 0:Dt:tiempo;
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


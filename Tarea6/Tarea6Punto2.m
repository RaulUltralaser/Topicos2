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
x_x = [-1; -1; 0; 0]; %Esto es en x
x_y = [0; 0; 0; 0]; %Esto es en y

% Inicializar el vector
x = zeros(2 * length(x_x), 1);

% Combinarlos para la simlacion
for i = 1:length(x_x)
    x(2*i-1) = x_x(i);
    x(2*i) = x_y(i);
end
x0=x;

%Datos de la simulacion
%periodo de muestreo
Dt=0.01;
tiempo=10; %segundos
iteraciones=tiempo/Dt;

%Simulacion
for k=1:iteraciones
   
    %Aproximación de Euler con Laplaciano
    x(:,k+1)=x(:,k)+Dt*(-KL*x(:,k));

  
end


t=linspace(0,tiempo,iteraciones+1);
% 
% Subplot para el consenso en el eje x
subplot(1, 2, 1);
hold on;
for i = 1:2:size(x, 1) % Consenso en x
     plot(t, x(i, :));
end
hold off;
xlabel('Tiempo (s)');
ylabel('\xi_{x}');
title('Consenso en el eje x');
grid on;

% Subplot para el consenso en el eje y
subplot(1, 2, 2);
hold on;
for i = 2:2:size(x, 1) % Consenso en y
    plot(t, x(i, :));
end
hold off;
xlabel('Tiempo (s)');
ylabel('\xi_{y}');
title('Consenso en el eje y');
grid on;


%% Esta area es para el punto d)

% Formación deseada (especificada en el mapa)
delta1=[0;2];
delta2=[-2;0];
delta3=[0;-2];
delta4=[2;0];
delta = [delta1;delta2;delta3;delta4]; % deltas concatenadas
P=zeros(2,1);

% Initialize the state matrix to store the simulation results
x_forma = zeros(size(x, 1), iteraciones + 1);
x_forma(:, 1) = x0;

% Simulacion
for k = 1:iteraciones
    %Aproximación de Euler con Laplaciano
    x_forma(:, k+1) = x_forma(:, k) + Dt * (-KL * (x_forma(:, k) - delta));
end

% Tiempo para ploteos
t = linspace(0, tiempo, iteraciones + 1);

% Errores de consenso
figure;
subplot(1, 2, 1);
hold on;
for i = 1:2:size(x_forma, 1) % Error de consenso en x
     plot(t, x_forma(i, :)-delta(i));
end
hold off;
xlabel('Tiempo (s)');
ylabel('e_{x}');
title('Error de consenso en el eje x');
grid on;
subplot(1, 2, 2);
hold on;
for i = 2:2:size(x_forma, 1) % Error de consenso en y
    plot(t, x_forma(i, :)-delta(i));
end
hold off;
xlabel('Tiempo (s)');
ylabel('e_{y}');
title('Error de consenso en el eje y');
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

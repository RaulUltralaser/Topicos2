clc
clearvars
close all

% Especificar la topología
% Definir la matriz de adyacencia A
agentes = 10;                  % número de agentes
lideres = 4;                   % número de líderes
seguidores = agentes - lideres; % número de seguidores
A = zeros(agentes, agentes);   % Inicializo la matriz de adyacencia

% Conexiones de los líderes a los seguidores
A(5, 1) = 1;  % l1 → f1
A(7, 2) = 1;  % l2 → f3
A(8, 3) = 1;  % l3 → f4
A(10, 4) = 1; % l4 → f6

% Conexiones entre los seguidores
A(6, 7) = 1;  % f3 → f2
A(7, 6) = 1;  % f2 → f3
A(6, 5) = 1;  % f1 → f2
A(5, 9) = 1;  % f5 → f1
A(9, 8) = 1;  % f4 → f5
A(9, 10) = 1; % f6 → f5
A(8, 9) = 1;  % f5 → f4
A(10, 8) = 1; % f4 → f6
A(9,5)=1;%agregue esta para mejor consenso f1 → f5

% Datos para simulación
Dt = 0.01;
tiempo = 40; % segundos
iteraciones = tiempo / Dt;

% Inicializar posiciones de los líderes y seguidores
ConvexHull = [-5;-5;  % L1
              5;-0.45;   % L2
              5;9.6;    % L3
              -5;5];  % L4

Followers = [0;-10;    % F1 
             0;-15;    % F2
             0;-20;    % F3
             0;10;     % F4
             0;15;     % F5
             0;20;];   % F6

xInit = [ConvexHull; Followers];

% Inicializar las posiciones
x = zeros(20, iteraciones);
x(:, 1) = xInit;

% Inicializar la matriz de control u
u = zeros(20, iteraciones); 

% Parámetros de control
alpha = 5;  % Escalar positivo para el control
beta = 1; % Escalar positivo para el término de signo

% Simulación
for k = 1:iteraciones
    % Definir la ley de control para los líderes (velocidad constante)
    for i = 1:lideres
        idx_i = 2*i-1:2*i; % Índice del líder i en las coordenadas x,y
        u(idx_i, k) = [1; cos(.2*Dt*k)];
        % u(idx_i, k) = [cos(Dt*k); sin(Dt*k)]; % Velocidad definida para líderes
    end
    
    % Definir la ley de control para los seguidores
    for i = 1:seguidores
        idx_i = 2*(i+lideres)-1:2*(i+lideres); % Índice del seguidor i en las coordenadas x,y
        u(idx_i, k) = 0; % Inicializar el control del seguidor i

        for j = 1:agentes
            if A(i+lideres, j) ~= 0  % Verificar si hay una conexión
                idx_j = 2*j-1:2*j;  % Índice del agente j en las coordenadas x,y
                control_term = A(i+lideres, j) * (x(idx_i, k) - x(idx_j, k));
                u(idx_i, k) = u(idx_i, k) - alpha * control_term - beta * sign(control_term);
            end
        end
    end
    
    % Aproximación de Euler de los estados
    x(:, k+1) = x(:, k) + Dt * (u(:, k));
end

%% Gráficas
% Crear la figura
figure;
hold on;

% Graficar las trayectorias de todos los agentes
for i = 1:agentes
    idx = 2*i-1:2*i;  % Índice del agente i en las coordenadas x,y
    if i<5
        plot(x(idx(1), :), x(idx(2), :), 'b-');  % Graficar las trayectorias
    else
        plot(x(idx(1), :), x(idx(2), :), 'k--');
    end
end

% Graficar los líderes como círculos azules y conectarlos para formar el convex hull
for i = 1:lideres
    idx = 2*i-1:2*i;
    plot(x(idx(1), 1), x(idx(2), 1), 'bo', 'MarkerFaceColor', 'b');        %Inicio
    plot(x(idx(1), 2000), x(idx(2), 2000), 'bo', 'MarkerFaceColor', 'b');  %20 segundos
    plot(x(idx(1), 4001), x(idx(2), 4001), 'bo', 'MarkerFaceColor', 'b');  %final
end

% Conectar los líderes con líneas para formar el convex hull inicial
plot([x(1,1) x(3,1)], [x(2,1) x(4,1)], 'b-');
plot([x(3,1) x(5,1)], [x(4,1) x(6,1)], 'b-');
plot([x(5,1) x(7,1)], [x(6,1) x(8,1)], 'b-');
plot([x(7,1) x(1,1)], [x(8,1) x(2,1)], 'b-');
% Conectar los líderes con líneas para formar el convex hull a los 20
% segundos
plot([x(1,4001) x(3,4001)], [x(2,4001) x(4,4001)], 'b-');
plot([x(3,4001) x(5,4001)], [x(4,4001) x(6,4001)], 'b-');
plot([x(5,4001) x(7,4001)], [x(6,4001) x(8,4001)], 'b-');
plot([x(7,4001) x(1,4001)], [x(8,4001) x(2,4001)], 'b-');

% Conectar los líderes con líneas para formar el convex hull a los 40
% segundos
plot([x(1,2000) x(3,2000)], [x(2,2000) x(4,2000)], 'b-');
plot([x(3,2000) x(5,2000)], [x(4,2000) x(6,2000)], 'b-');
plot([x(5,2000) x(7,2000)], [x(6,2000) x(8,2000)], 'b-');
plot([x(7,2000) x(1,2000)], [x(8,2000) x(2,2000)], 'b-');

% Graficar las posiciones iniciales de los seguidores como cuadrados de color gris
for i = 1:seguidores
    idx = 2*(i+lideres)-1:2*(i+lideres);
    plot(x(idx(1), 1), x(idx(2), 1), 's', 'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerEdgeColor', 'k'); %Inicio
    plot(x(idx(1), 2000), x(idx(2), 2000), 's', 'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerEdgeColor', 'r');     %20 segundos
end

% Graficar las posiciones finales de todos los agentes como cuadrados rojos
for i = 1:agentes
    idx = 2*i-1:2*i;
    if i>5
        plot(x(idx(1), end), x(idx(2), end), 's', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
    end
end

% Etiquetas y ajustes
xlabel('Posición en X');
ylabel('Posición en Y');
title('Trayectorias de los agentes (Líderes y Seguidores)');
axis equal
grid on;
hold off;

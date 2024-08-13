clc
clearvars
close all

%Especificar la topologia
% Definir la matriz de adyacencia A
agentes=10;                  %numero agentes
lideres=4;                   %numero lideres
seguidores=agentes-lideres;  %numero de seguidores
A = zeros(agentes, agentes); %Inicializo la matriz de adyacencia

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
A(8, 9) = 1; % f5 → f4
A(10, 8) = 1; % f4 → f6
A(9,5)=1;     %agregue esta para mejor consenso f1 → f5

%Datos para simulacion
Dt=0.01;
tiempo=7; %segundos
iteraciones=tiempo/Dt;

%Inicializo datos
%posiciones de los lideres en x;y
ConvexHull=[-5;-5; %L1
            5;-5; %L2
            5;5; %L3
            -5;5];%L4
%posiciones iniciales de seguidores x;y
Followers=[0;-10;    %F1 
           0;-15;  %F2
           0;-20;  %F3
           0;10;     %F4
           0;15;   %F5
           0;20;];   %F6
xInit=[ConvexHull;Followers];

%Inicializar las posiciones
x=zeros(20,iteraciones);
x(:,1)=xInit;

% Inicializar la matriz de control u
u = zeros(20, iteraciones);  % 20 porque hay 10 agentes, y cada uno tiene 2 coordenadas (x, y)

%Simulacion
for k=1:iteraciones
    % Definir la ley de control para los seguidores
    for i = 1:seguidores
        idx_i = 2*(i+lideres)-1:2*(i+lideres); % Índice del seguidor i en las coordenadas x,y
        u(idx_i,k) = 0; % Inicializar el control del seguidor i

        for j = 1:agentes
            if A(i+lideres, j) ~= 0  % Verificar si hay una conexión
                idx_j = 2*j-1:2*j;  % Índice del agente j en las coordenadas x,y
                u(idx_i,k) = u(idx_i,k) - A(i+lideres, j) * (x(idx_i, k) - x(idx_j, k));
            end
        end
    end
    %aproximacion de Euler de los estados
    x(:,k+1)=x(:,k)+Dt*(u(:,k));
end

%% Graficas
% Crear la figura
figure;
hold on;

% Graficar las trayectorias de los seguidores
for i = 1:seguidores
    idx = 2*(i+lideres)-1:2*(i+lideres);  % Índice del seguidor i en las coordenadas x,y
    plot(x(idx(1), :), x(idx(2), :), 'k--');  % Graficar las trayectorias de los seguidores
end

% Graficar los líderes como círculos azules y conectarlos para formar el convex hull
plot(x(1, 1), x(2, 1), 'bo', 'MarkerFaceColor', 'b');  % L1
plot(x(3, 1), x(4, 1), 'bo', 'MarkerFaceColor', 'b');  % L2
plot(x(5, 1), x(6, 1), 'bo', 'MarkerFaceColor', 'b');  % L3
plot(x(7, 1), x(8, 1), 'bo', 'MarkerFaceColor', 'b');  % L4

% Conectar los líderes con líneas para formar el convex hull
plot([x(1,1) x(3,1)], [x(2,1) x(4,1)], 'b-');
plot([x(3,1) x(5,1)], [x(4,1) x(6,1)], 'b-');
plot([x(5,1) x(7,1)], [x(6,1) x(8,1)], 'b-');
plot([x(7,1) x(1,1)], [x(8,1) x(2,1)], 'b-');

% Graficar las posiciones iniciales de los seguidores como cuadrados de un color
for i = 1:seguidores
    idx = 2*(i+lideres)-1:2*(i+lideres);
    plot(x(idx(1), 1), x(idx(2), 1), 's', 'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerEdgeColor', 'k');  % Color gris
end

% Graficar las posiciones finales de los seguidores como cuadrados rojos
for i = 1:seguidores
    idx = 2*(i+lideres)-1:2*(i+lideres);
    plot(x(idx(1), end), x(idx(2), end), 's', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');  % Color rojo
end

% Etiquetas y ajustes
xlabel('Posición en X');
ylabel('Posición en Y');
title('Trayectorias de los agentes');
axis equal
grid on;
hold off;



% Crear el objeto digraph
G = digraph(A');

% Etiquetas para los nodos (líderes y seguidores)
nombres_nodos = {'L1', 'L2', 'L3', 'L4', 'F1', 'F2', 'F3', 'F4', 'F5', 'F6'};

% Graficar el grafo dirigido
figure;
plot(G, 'Layout', 'layered', 'NodeLabel', nombres_nodos, 'ArrowSize', 12);
title('Grafo dirigido de la matriz de adyacencia');



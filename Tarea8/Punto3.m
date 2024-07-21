clc;
clearvars;
close all;

l = 0.5; % Distancia del centro del robot al punto de control
k=2;     % Ganancia de convergencia
sigma=.5;% Rango de evasion
lambda=2; 

%Condiciones iniciales
alphaInicial = [2; 1; 1.95; .95; 3; -1; 2; -2]; % Cada columna es [x; y] de un robot
Angulo = [90; 0; -45; -90]; % Angulos dados en grados

% Laplaciano grafo no dirigido (completamente dirigido como en el paper)
L = [3 -1 -1 -1;
    -1  3 -1 -1;
    -1 -1  3 -1;
    -1 -1 -1  3];
I2 = eye(2);

% Datos de la simulacion
Dt = 0.01; % Periodo de muestreo
tiempo = 30; % Duración de la simulacion en segundos
iteraciones = tiempo / Dt;

%Inicializacion de valores
alpha=zeros(8,iteraciones+1);
alpha(:,1)=alphaInicial;
theta = zeros(4, iteraciones+1);
theta(:, 1) = deg2rad(Angulo);
%Simulacion
for k=1:iteraciones

    M1 = [cos(theta(1,k)) -l*sin(theta(1,k));
         sin(theta(1,k)) l*cos(theta(1,k))];
    M2 = [cos(theta(2,k)) -l*sin(theta(2,k));
         sin(theta(2,k)) l*cos(theta(2,k))];
    M3 = [cos(theta(3,k)) -l*sin(theta(3,k));
         sin(theta(3,k)) l*cos(theta(3,k))];
    M4 = [cos(theta(4,k)) -l*sin(theta(4,k));
         sin(theta(4,k)) l*cos(theta(4,k))];

    M = blkdiag(M1, M2, M3, M4);


    J_o()=

    J_mas_o=J_o'/(J_o*J_o');
    

    e_dot_prima_o=J_o*J_mas_r*e_dot_r;
    alpha_dot_prima_o=J_mas_o*e_dot_prima_o;

    J_r=I2;
    N_o=I2-J_mas_o*J_o;
    e_dot_r
    alpha_dot_ro=J_r*N_o+(e_dot_r+r_dot-J_r*J_mas_o*e_dot_prima_o);

    %Ley de control
    u=M\(alpha_dot_prima_o+alpha_dot_ro);

    alpha(:,k+1) = alpha(:,k) + Dt * (M * u);

    

end 

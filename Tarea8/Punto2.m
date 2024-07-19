clc;
clearvars;
close all;

% Parámetros iniciales  tienen que ser mayores a cero
l = 0.5; % Distancia del centro del robot al punto de control
k_x = 1; % Ganancia del controlador para error en x
k_y = 1; % Ganancia del controlador para error en y
k_theta = 1; % Ganancia del controlador para error en theta 
kappa = 1; % Constante para el termino feed-forward 
n=2;       %Duración del Bump
tau= 1;    %Constante para el termino feed-forward
R = 1.5; % Rango de detección del sensor
r = 0.5; % Distancia mínima aceptable a los obstáculos
agentes=4; %Número de agentes

% Laplaciano grafo no dirigido
L = [2 -1  0 -1;
    -1  2 -1  0;
     0 -1  2 -1;
    -1  0 -1  2];
I = eye(2);

% Inicializar condiciones iniciales para 4 robots
z = [2, 1;
     1.95, 0.95;
     3, -1;
     2, -2]; % Cada fila es [x, y] de un robot
theta = deg2rad([90, 0, -45, -90]); % Ángulos iniciales en radianes


% Datos de la simulación
Dt = 0.01; % Periodo de muestreo
tiempo = 30; % Duración de la simulación en segundos
iteraciones = tiempo / Dt;

%inicializo v para poder calcular v_d_i
v_star=zeros(agentes,iteraciones);
w_star=zeros(agentes,iteraciones);

% Inicializar trayectorias
z_hist = zeros(4, 2, iteraciones+1);
theta_hist = zeros(4, iteraciones+1);
z_hist(:, :, 1) = z;
theta_hist(:, 1) = theta;

% Simulación
for k = 1:iteraciones
    
      M1=[cos(theta(k)) 0;
         sin(theta(k)) 0;
         0               1];
      M2=[cos(theta(k)) 0;
         sin(theta(k)) 0;
         0               1];
      M3=[cos(theta(k)) 0;
         sin(theta(k))  0;
         0               1];
      M4=[cos(theta(k)) 0;
         sin(theta(k))  0;
         0               1];
      
      M = blkdiag(M1, M2, M3, M4);
      
      %TODO: no sé como chingados hacerle para que fijar el tiempo k0
      for i=1:agentes
          for j=1:agentes
            if abs(v_star(i,k)-v_star(j,k)) >= kappa && i~=j
                v_d_i=exp(-tau);
            else
                v_d_i=bumpFunc(tau,Dt,k,k0,n);
            end
          end
      end

      gamma=gammaFunc(theta_io,p_io);
        
      

      q_iu=(1-gamma)*q_id+gamma*q_ic;
      
      errores=[cos(theta(k)) sin(theta(k)) 0;
               -sin(theta(k)) cos(theta(k)) 0;
               0               0            1]*(q_iu-q_star);
    
      v_star=v_d_i*cos(e_theta)+k_x*e_x;
      w_star=k_y*v_d_i*e_y*(sin(e_theta)/e_theta)+k_theta*e_theta;

      u_i_star=[v_i_star w_i_star]';

      
    
end

% t = linspace(0, tiempo, iteraciones);
% 
% % Gráfica de resultados
% figure;
% hold on;
% colors = ['r', 'b', 'g', 'k'];
% for i = 1:4
%     plot(squeeze(z_hist(i, 1, :)), squeeze(z_hist(i, 2, :)), colors(i));
%     quiver(z_hist(i, 1, 1), z_hist(i, 2, 1), cos(theta_hist(i, 1)), sin(theta_hist(i, 1)), 0.1, colors(i), 'LineWidth', 0.1, 'MaxHeadSize', 1);
%     quiver(z_hist(i, 1, end), z_hist(i, 2, end), cos(theta_hist(i, end)), sin(theta_hist(i, end)), 0.1, colors(i), 'LineWidth', 0.1, 'MaxHeadSize', 2);
% end
% xlabel('x');
% ylabel('y');
% title('Trayectoria del robot no holónomo');
% hold off;
% grid on;


function sigma=bumpFunc(tau,Dt,k,k0,n)

    t=Dt*k;   %Tiempo de la simulación
    t0=Dt*k0; %Tiempo en el que se dispara la función bump
    if t0 < t && t <= n + t0
        sigma = exp(-tau / (1 - ((t - t0) / n)^2));
    else
        sigma = 0;
    end
end

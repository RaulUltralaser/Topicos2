clc;
clearvars;
close all;

% Parámetros iniciales  tienen que ser mayores a cero
l = 0.5; % Distancia del centro del robot al punto de control
k_x = 1; % Ganancia del controlador para error en x
k_y = 1; % Ganancia del controlador para error en y
k_theta = 1; % Ganancia del controlador para error en theta 
kappa = 1; % Constante para el termino feed-forward 
n=10;       %Duración del Bump
tau= 1;    %Constante para el termino feed-forward
R = 1.5; % Rango de detección del sensor
r = 1; % Distancia mínima aceptable a los obstáculos [x y]
agentes=4; %Número de agentes

% Laplaciano grafo no dirigido
L = [2 -1  0 -1;
    -1  2 -1  0;
     0 -1  2 -1;
    -1  0 -1  2];
I1 = eye(1);
I2 = eye(2);
%Estos son los Di (Es decir, la raiz del grado de cada nodo
%, pero todos son iguales entonces solo pongo uno)
Di=[sqrt(2)];
D=Di*eye(agentes);
%Laplaciano normalizado
Ld=inv(D)\L*inv(D); %inv(D)*L*inv(D)

kronp=kron(Ld,I2);
krontheta=kron(Ld,I1);

% Inicializar condiciones iniciales para 4 robots
z = [2;1;1.95; 0.95;3; -1; 2; -2]; % Cada fila es [x, y] de un robot
thetaInicial = deg2rad([90, 0, -45, -90]); % Ángulos iniciales en radianes


% Datos de la simulación
Dt = 0.01; % Periodo de muestreo
tiempo = 30; % Duración de la simulación en segundos
iteraciones = tiempo / Dt;

%inicializo v para poder calcular v_d_i
v_star=zeros(agentes,iteraciones);
w_star=zeros(agentes,iteraciones);

%Incicializo todos los valores
p=zeros(8,iteraciones+1);
p(:,1)=z;
q_star=zeros(12,iteraciones);
q_star(:,1)=[z(1:2);thetaInicial(1);z(3:4);thetaInicial(2);z(5:6);thetaInicial(3);z(7:8);thetaInicial(4);];
q_id=zeros(12,iteraciones);
q_ic=zeros(12,iteraciones);
q_iu=zeros(12,iteraciones);
p_io=zeros(8,iteraciones);
p_ia=p_io;
p_ic=p_io;
theta=zeros(4,iteraciones+1);
theta(:,1)=thetaInicial;
theta_io=zeros(4,iteraciones);
theta_ic=zeros(4,iteraciones);
theta_bario=zeros(4,iteraciones);
e=zeros(12,iteraciones);
%HAGO USO DE u_i_star antes de calcularlo entonces lo inicializo en zeros
u_i_star=zeros(8,iteraciones+1);
t0_establecido=false;
% Simulación
for k = 1:iteraciones
        
      %Aproximación de Euler de p estrella
      p(:,k+1)=p(:,k)+Dt*(-kronp*p(:,k));
        
      %Aproximacion de Euler de theta estrella
      theta(:,k+1)=theta(:,k)+Dt*(-krontheta*theta(:,k));

     sump=zeros(8,1);
     % Bucle para calcular la suma de p
      for i = 1:agentes
        for j = 1:agentes
            % Verificar si existe una conexión entre agentes usando L
            if L(i, j) == -1 
                % Índices en sump para el agente i
                switch i
                    case 1
                        idx_i = 1:2; % Índices para el agente 1 (sump1_x, sump1_y)
                    case 2
                        idx_i = 3:4; % Índices para el agente 2 (sump2_x, sump2_y)
                    case 3
                        idx_i = 5:6; % Índices para el agente 3 (sump3_x, sump3_y)
                    case 4
                        idx_i = 7:8; % Índices para el agente 4 (sump4_x, sump4_y)
                end
                % Sumar las coordenadas x e y del agente j a las correspondientes del agente i
                sump(idx_i) = sump(idx_i) + p(idx_i,k+1);
            end
        end
     end
    
      sumtheta=zeros(4,1);
      for i = 1:agentes
            for j = 1:agentes
                %Verifico si existe una conexion entre agentes usando L
                if L(i, j) == -1
                    sumtheta(i) = sumtheta(i)+theta(j,k+1);
                end
            end
      end
     
      %Calculo q_id
      q_id(:,k)=[(Di\sump*inv(Di))' (Di\sumtheta*inv(Di))']';
      %Lo acomodo para que queden como [x y theta x y theta...]
      q_id(:,k)=[q_id(1:2,k);q_id(3,k);q_id(4:5,k);q_id(6,k);q_id(7:8,k);q_id(9,k);q_id(10:11,k);q_id(12,k)];
    

      %Para calcular el vector de avoidance checo la distancia entre
      %agentes uno por uno
      for i= 1:agentes
          switch i
                    case 1
                        id_i = 1:2; % Índices para el agente 1 
                    case 2
                        id_i = 3:4; % Índices para el agente 2 
                    case 3
                        id_i = 5:6; % Índices para el agente 3 
                    case 4
                        id_i = 7:8; % Índices para el agente 4 
           end
          for j= 1:agentes
              if i~= j
                switch j
                    case 1
                        id_j = 1:2; % Índices para el agente 1 
                    case 2
                        id_j = 3:4; % Índices para el agente 2 
                    case 3
                        id_j = 5:6; % Índices para el agente 3 
                    case 4
                        id_j = 7:8; % Índices para el agente 4 
                end
                p_i=p(id_i,k);
                p_o=p(id_j,k);
                if norm(p_i-p_o)<r
                    p_io(id_i,k)=p_i-p_o;
                end
              end
          end
      end
      
      %Y el bearing angulo lo calculo con la funcin atan2
      for i=1:agentes
          switch i
            case 1
                id = [1 2]; % Índices para el agente 1 (sump1_x, sump1_y)
            case 2
                id = [3 4]; % Índices para el agente 2 (sump2_x, sump2_y)
            case 3
                id = [5 6]; % Índices para el agente 3 (sump3_x, sump3_y)
            case 4
                id = [7 8]; % Índices para el agente 4 (sump4_x, sump4_y)
           end
        theta_io(i,k)=atan2(-p_io(id(2)),-p_io(id(1)));
      end 
      
      %Calculo el angulo relativo entre el robot y el obstaculo
      theta_bario(:,k)=theta(:,k+1)-theta_io(:,k);

      % Calculo vector atractor
      % Bucle para calcular la suma de p
      for i = 1:agentes
        for j = 1:agentes
            % Verificar si existe una conexión entre agentes usando L
            if L(i, j) == -1 
                % Índices en sump para el agente i
                switch i
                    case 1
                        idx_i = 1:2; % Índices para el agente 1 (sump1_x, sump1_y)
                    case 2
                        idx_i = 3:4; % Índices para el agente 2 (sump2_x, sump2_y)
                    case 3
                        idx_i = 5:6; % Índices para el agente 3 (sump3_x, sump3_y)
                    case 4
                        idx_i = 7:8; % Índices para el agente 4 (sump4_x, sump4_y)
                end
                % Sumar las coordenadas x e y del agente j a las correspondientes del agente i
                sump(idx_i) = sump(idx_i) + p(idx_i,k+1);
            end
        end
     end
      p_ia(:,k)=(Di\sump*inv(Di))-p(:,k+1);
      
      %Calculo el final del vector de avoidance que corresponde a p_ic
      for i=1:agentes
        switch i
            case 1
                id = 1:2; % Índices para el agente 1 
            case 2
                id = 3:4; % Índices para el agente 2 
            case 3
                id = 5:6; % Índices para el agente 3 
            case 4
                id = 7:8; % Índices para el agente 4 
        end
        p_ic(id,k)=find_p_ic(p_io(id,k),p_ia(id,k));
      end

      %Calculo el angulo deseado
      for i=1:agentes
          switch i
            case 1
                id = [1 2]; % Índices para el agente 1 (sump1_x, sump1_y)
            case 2
                id = [3 4]; % Índices para el agente 2 (sump2_x, sump2_y)
            case 3
                id = [5 6]; % Índices para el agente 3 (sump3_x, sump3_y)
            case 4
                id = [7 8]; % Índices para el agente 4 (sump4_x, sump4_y)
           end
        theta_ic(i,k)=atan2(p_ic(id(2),k)-p(2,k+1),p_ic(id(1),k)-p(1,k+1));
      end
      
      %Calculo q_ic y lo acomodo de modo [x_ic,y_ic,theta_ic]
      q_ic(:,k)=[p_ic(1:2,k);theta(1,k);p_ic(3:4,k);theta(2,k);p_ic(5:6,k);theta(3,k);p_ic(7:8,k);theta(4,k)];

      %Calculo de la funcion gamma para q_iu
      for i=1:agentes
        switch i
            case 1
                id = 1:2; % Indices para el agente 1 necesarias para gamma
                idq = 1:3;% Indices para el agente 1 para q
            case 2
                id = 3:4; % Indices para el agente 2 necesarias para gamma 
                idq = 4:6;% Indices para el agente 2 para q
            case 3
                id = 1:2; % Indices para el agente 3 necesarias para gamma
                idq = 7:9;% Indices para el agente 3 para q
            case 4
                id = 1:2; % Indices para el agente 4 necesarias para gamma
                idq = 10:12;% Indices para el agente 4 para q
        end
        gamma=gammaFunc(theta_bario(i,k),p_io(id,k),R,r);
        q_iu(id,k)=(1-gamma)*q_id(id,k)+gamma*q_ic(id,k);
      end
      
      %Genero la M
      M1=[cos(theta(1,k+1)) 0;
          sin(theta(1,k+1)) 0;
          0               1];
      M2=[cos(theta(2,k+1)) 0;
          sin(theta(2,k+1)) 0;
          0               1];
      M3=[cos(theta(3,k+1)) 0;
          sin(theta(3,k+1)) 0;
          0               1];
      M4=[cos(theta(4,k+1)) 0;
          sin(theta(4,k+1)) 0;
          0               1];
      M=blkdiag(M1,M2,M3,M4);
      %Aproximacion de Euler de q estrella
      q_star(:,k+1)=q_star(:,k)+Dt*(M*u_i_star(:,k));
      
      %Calculo de los errores por separado
      e1=[cos(theta(1,k)) sin(theta(1,k)) 0;
         -sin(theta(1,k)) cos(theta(1,k)) 0;
          0               0               1]*(q_iu(1:3,k)-q_star(1:3,k+1));
      e2=[cos(theta(2,k)) sin(theta(2,k)) 0;
         -sin(theta(2,k)) cos(theta(2,k)) 0;
          0               0               1]*(q_iu(4:6,k)-q_star(4:6,k+1));
      e3=[cos(theta(3,k)) sin(theta(3,k)) 0;
         -sin(theta(3,k)) cos(theta(3,k)) 0;
          0               0               1]*(q_iu(7:9,k)-q_star(7:9,k+1));
      e4=[cos(theta(4,k)) sin(theta(4,k)) 0;
         -sin(theta(4,k)) cos(theta(4,k)) 0;
          0               0               1]*(q_iu(10:12,k)-q_star(10:12,k+1));
    
      %Junto los valores y los guardo para graficarlos
      e(:,k)=[e1;e2;e3;e4];
      
      for i=1:agentes
          switch i
              case 1
                xe=1;ye=2;thetae=3;
              case 2
                xe=4;ye=5;thetae=6;
              case 3
                xe=7;ye=8;thetae=9;
              case 4
                xe=10;ye=11;thetae=12;
          end
          for j=1:agentes
            if abs(v_star(i,k)-v_star(j,k)) >= kappa && i~=j
                v_d_i=exp(-tau);
                t0_establecido=false;
            else
                t=k*Dt;
                if ~t0_establecido
                    % Establecer t0 la primera vez que entra en el else
                    t0 = t;
                    t0_establecido = true;
                end
                v_d_i=bumpFunc(tau,t,t0,n);
            end
          end
          v_star(i,k)=v_d_i*cos(e(thetae,k))+k_x*e(xe,k);
          if e(thetae,k)==0
              operation=1;
          else
              operation=(sin(e(thetae,k))/e(thetae,k));
          end
          w_star(i,k)=k_y*v_d_i*e(ye,k)*operation+k_theta*e(thetae,k);
      end

      %Calculo y acomodo u_i_star para graficarlos despues
      u_i_star(:,k+1)=[v_star(1,k) w_star(1,k) v_star(2,k) w_star(2,k) v_star(3,k) w_star(3,k) v_star(4,k) w_star(4,k)]';
end

t = linspace(0, tiempo, iteraciones+1);
% 
% Gráfica de resultados
figure;
hold on;
plot(q_star(1,:),q_star(2,:))
quiver(q_star(1, 1), q_star(2, 1), cos(q_star(3,1)), sin(q_star(3,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(q_star(1, end), q_star(2, end), cos(q_star(3,end)), sin(q_star(3,end)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 2);
plot(q_star(4,:),q_star(5,:))
quiver(q_star(4, 1), q_star(5, 1), cos(q_star(6,1)), sin(q_star(6,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(q_star(4, end), q_star(5, end), cos(q_star(6,end)), sin(q_star(6,end)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 2);
plot(q_star(7,:),q_star(8,:))
quiver(q_star(7, 1), q_star(8, 1), cos(q_star(9,1)), sin(q_star(9,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(q_star(7, end), q_star(8, end), cos(q_star(9,end)), sin(q_star(9,end)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 2);
plot(q_star(10,:),q_star(11,:))
quiver(q_star(10, 1), q_star(11, 1), cos(q_star(12,1)), sin(q_star(12,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(q_star(10, end), q_star(11, end), cos(q_star(12,end)), sin(q_star(12,end)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 2);
% plot(p(1,:),p(2,:))
% plot(p(3,:),p(4,:))
% plot(p(5,:),p(6,:))
% plot(p(7,:),p(8,:))
xlabel('x');
ylabel('y');
title('Trayectoria del robot no holónomo');
hold off
grid on


t = linspace(0, tiempo, iteraciones);
figure 
subplot(1,3,1)
hold on 
plot(t,e(1,:))
plot(t,e(4,:))
plot(t,e(7,:))
plot(t,e(10,:))
xlabel('t');
ylabel('error');
title('Error en el eje x');
grid on
hold off
subplot(1,3,2)
hold on
plot(t,e(2,:))
plot(t,e(5,:))
plot(t,e(8,:))
plot(t,e(11,:))
xlabel('t');
ylabel('error');
title('Error en el eje y');
grid on 
hold off
subplot(1,3,3)
hold on
plot(t,e(3,:))
plot(t,e(6,:))
plot(t,e(9,:))
plot(t,e(12,:))
xlabel('t');
ylabel('error');
title('Error en el angulo theta');
grid on 
hold off

figure
subplot(1,2,1)
plot(t,v_star)
xlabel('t');
ylabel('v');
title('Ley de control v');
grid on
subplot(1,2,2)
plot(t,w_star)
xlabel('t');
ylabel('w');
title('Ley de control w');
grid on

function p_io_perp = find_p_ic(p_io, p_ia)
    % Calcular el vector perpendicular
    p_io_perp = [-p_io(2), p_io(1)];
    
    % Verificar el producto interno
    if dot(p_ia, p_io_perp) < 0
        % Invertir el signo de p_io_perp si el producto interno es negativo
        p_io_perp = -p_io_perp;
    end
end

function gamma = gammaFunc(theta_io, p_io, R, r)
 
    % Calcular el valor absoluto de theta_io
    bar_theta_io = abs(theta_io);
    
    % Calcular el valor de dm
    if sin(bar_theta_io) == 0
        d_m = R; % Evitar división por cero
    else
        d_m = min(R, r / sin(bar_theta_io));
    end
    
    % Calcular la norma de p_io
    norm_p_io = norm(p_io);
    
    % Determinar el valor de gamma
    if bar_theta_io >= pi/2 || norm_p_io > d_m
        gamma = 0;
    else
        gamma = 1;
    end
end

function sigma=bumpFunc(tau,t,t0,n)
    %Se considera que el tiempo nunca sera menor a 0
    if t0 <= t && t < n + t0
        sigma = exp(-tau / (1 - ((t - t0) / n)^2));
    else
        sigma = 0;
    end
end

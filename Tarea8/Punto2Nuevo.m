clc
clearvars
close all

%Parametros iniciales
d=.5;        %Distancia del robot al punto de control
l=0.5;      %Distancia del punto de control al agente virtual
agentes=4;  %Numero de agentes
r=0.2;  %Distancia de seguridad
k1=10;     %Ganancia de evasion
k2=2;     %Ganancia de consenso
R=1.5;    % Rango de detección del sensor
delta=.1;


%Matriz laplaciana 
L=[2 -1  0 -1;
  -1  2 -1  0;
   0 -1  2 -1;
  -1  0 -1  2];
I2=eye(2);
Jr=I2;
kronL=kron(L,I2);

%Estos son los Di (Es decir, la raiz del grado de cada nodo
%, pero todos son iguales entonces solo pongo uno)
Di=[sqrt(2)];
D=Di*eye(agentes);

%Posiciones iniciales robots reales, elegidas al azar
x=[1 2 3 2];
y=[1 1 -1 -2];
Theta=deg2rad([0 0 90 180]);
%Posiciones iniciales de los puntos de control
p1=[x(1)+d*round(cos(Theta(1))); y(1)+d*round(sin(Theta(1)))];
p2=[x(2)+d*round(cos(Theta(2))); y(2)+d*round(sin(Theta(2)))];
p3=[x(3)+d*round(cos(Theta(3))); y(3)+d*round(sin(Theta(3)))];
p4=[x(4)+d*round(cos(Theta(4))); y(4)+d*round(sin(Theta(4)))];
pInit=[p1;p2;p3;p4];

%Formacion especificada (un rombo)
z1=l*[round(cos((3*pi)/2)); round(sin((3*pi)/2))];
z2=l*[round(cos(2*pi)); round(sin(2*pi))];
z3=l*[round(cos(pi/2)); round(sin(pi/2))];
z4=l*[round(cos(3*pi)); round(sin(pi))];
z=[z1;z2;z3;z4];

%Posiciones iniciales agentes virtuales
p_star1=p1+z1;
p_star2=p2+z2;
p_star3=p3+z3;
p_star4=p4+z4;
p_starInit=[p_star1;p_star2;p_star3;p_star4];

%Condiciones de la simulacion
Dt = 0.01; % Periodo de muestreo
tiempo = 10; % Duracion de la simulacion en segundos
iteraciones = tiempo / Dt;

%Inicialización de parametros
p_star=zeros(agentes*2,iteraciones+1); %Agentes virtuales
p_star(:,1)=p_starInit;
p=zeros(agentes*2,iteraciones);    %Puntos de control
p(:,1)=pInit;
theta=zeros(agentes,iteraciones);
theta(:,1)=Theta;

q_star=zeros(3*agentes,iteraciones+1);
q_star(:,1)=[p_starInit;Theta'];
q_id=zeros(3*agentes,iteraciones);
q_ic=zeros(3*agentes,iteraciones);
q_iu=zeros(3*agentes,iteraciones);
theta_io=zeros(agentes,iteraciones);
theta_bario=zeros(agentes,iteraciones);
theta_ic=zeros(agentes,iteraciones);
p_io=zeros(2*agentes,iteraciones);
p_ia=p_io;
p_ic=p_io;
e=zeros(12,iteraciones);


v_hist=zeros(agentes,iteraciones);
w_hist=zeros(agentes,iteraciones);
%Simulacion
for k=1:iteraciones
%% Calculo de q_id
    sump=zeros(8,1);
     % Bucle para calcular la suma de p
      for i = 1:agentes
        for j = 1:agentes
            % Verificar si existe una conexión entre agentes usando L
            if L(i, j) == -1 
                % Sumar las coordenadas x e y del agente j a las correspondientes del agente i
                sump(2*i-1:2*i) = sump(2*i-1:2*i) + q_star(2*i-1:2*i,k);
            end
        end
     end
    
      sumtheta=zeros(4,1);
      for i = 1:agentes
            for j = 1:agentes
                %Verifico si existe una conexion entre agentes usando L
                if L(i, j) == -1
                    sumtheta(i) = sumtheta(i)+theta(j,k);
                end
            end
      end
     
      %Calculo q_id 
      q_id(:,k)=[(Di\sump*inv(Di))' (Di\sumtheta*inv(Di))']';
      
%% Calculo de q_ic
    %Para calcular el vector de avoidance checo la distancia entre
    %puntos de control de cada robot
      for i= 1:agentes
          for j= 1:agentes
              if i~= j
                p_i=q_star(2*i-1:2*i,k)-z(2*i-1:2*i); %porque p_i=p_i_star-z_i
                p_o=q_star(2*j-1:2*j,k)-z(2*j-1:2*j); %porque p_j=p_j_star-z_j
                if norm(p_i-p_o)<r
                    p_io(2*i-1:2*i,k)=p_i-p_o;
                end
              end
          end
      end

      %Y la orientacion del angulo theta_io lo calculo con la funcin atan2
      for i=1:agentes
        theta_io(i,k)=atan2(-p_io(2*i),-p_io(i));
      end 

      %Recordar que el punto de control y el agente virtual comparten el
      %mismo angulo
      theta_bario(:,k)=q_star(9:12,k)-theta_io(:,k);

      %Vector de atraccion con los agentes virtuales
      sump=zeros(8,1);
     % Bucle para calcular la suma de p
      for i = 1:agentes
        for j = 1:agentes
            % Verificar si existe una conexión entre agentes usando L
            if L(i, j) == -1 
                % Sumar las coordenadas x e y del agente j a las correspondientes del agente i
                sump(2*i-1:2*i) = sump(2*i-1:2*i) + q_star(2*i-1:2*i,k);
            end
        end
      end
      %Este es mi vector de atracción
      p_ia(:,k)=(Di\sump*inv(Di))-q_star(1:8,k);
    
      %Calculo el final del vector de avoidance que corresponde a p_ic
      for i=1:agentes
        p_ic(2*i-1:2*i,k)=find_p_ic(p_io(2*i-1:2*i,k),p_ia(2*i-1:2*i,k));
      end

      %Calculo el angulo deseado theta_ic
      for i=1:agentes
        theta_ic(i,k)=atan2(p_ic(2*i,k)-(q_star(2*i,k)-z(2*i)),p_ic(2*i-1,k)-(q_star(2*i-1,k)-z(2*i-1)));
      end

      q_ic=[p_ic(:,k);theta_ic(:,k)];

%% Calculo q_iu
        for i=1:agentes
            gamma=gammaFunc(theta_bario(i,k),p_io(2*i-1:2*i,k),R,r);
            q_iu(2*i-1:2*i,k)=(1-gamma)*q_id(2*i-1:2*i,k)+gamma*q_ic(2*i-1:2*i,k);
        end
%% Mi dinamica es esta
      %Genero la M
      M1=[cos(theta(1,k)) 0;
          sin(theta(1,k)) 0];
      M2=[cos(theta(2,k)) 0;
          sin(theta(2,k)) 0];
      M3=[cos(theta(3,k)) 0;
          sin(theta(3,k)) 0];
      M4=[cos(theta(4,k)) 0;
          sin(theta(4,k)) 0];
      Ms=blkdiag(M1,M2,M3,M4);
      Mi=blkdiag([0 1],[0 1],[0 1],[0 1]);
      M=[Ms;Mi]
      %Aproximacion de Euler de q estrella pero como [x;y;theta]
      q_star(:,k+1)=q_star(:,k)+Dt*(M*u_i_star(:,k));
end

% figure
% hold on
% plot(alpha(1,:),alpha(2,:))
% quiver(alpha(1, 1), alpha(2, 1), cos(theta(1,1)), sin(theta(3,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
% quiver(alpha(1, end), alpha(2, end), cos(theta(1,end)), sin(theta(3,end)), 0.1, 'g', 'LineWidth', .1, 'MaxHeadSize', 2);
% plot(alpha(3,:),alpha(4,:))
% quiver(alpha(3, 1), alpha(4, 1), cos(theta(2,1)), sin(theta(2,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
% quiver(alpha(3, end), alpha(4, end), cos(theta(2,end)), sin(theta(2,end)), 0.1, 'g', 'LineWidth', .1, 'MaxHeadSize', 2);
% plot(alpha(5,:),alpha(6,:))
% quiver(alpha(5, 1), alpha(6, 1), cos(theta(3,1)), sin(theta(3,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
% quiver(alpha(5, end), alpha(6, end), cos(theta(3,end)), sin(theta(3,end)), 0.1, 'g', 'LineWidth', .1, 'MaxHeadSize', 2);
% plot(alpha(7,:),alpha(8,:))
% quiver(alpha(7, 1), alpha(8, 1), cos(theta(4,1)), sin(theta(4,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
% quiver(alpha(7, end), alpha(8, end), cos(theta(4,end)), sin(theta(4,end)), 0.1, 'g', 'LineWidth', .1, 'MaxHeadSize', 2);
% xlabel('x');
% ylabel('y');
% title('Trayectoria de los robots no holónomos');
% grid on
% hold off
% 
% figure
% hold on
% plot(alphav(1,:),alphav(2,:))
% plot(alphav(3,:),alphav(4,:))
% plot(alphav(5,:),alphav(6,:))
% plot(alphav(7,:),alphav(8,:))
% grid on
% hold off
% 
% t = linspace(0, tiempo, iteraciones);
% figure 
% subplot(1,2,1)
% hold on 
% plot(t,e(1,:))
% plot(t,e(3,:))
% plot(t,e(5,:))
% plot(t,e(7,:))
% xlabel('t');
% ylabel('error');
% title('Error en el eje x');
% grid on
% hold off
% subplot(1,2,2)
% hold on
% plot(t,e(2,:))
% plot(t,e(4,:))
% plot(t,e(6,:))
% plot(t,e(8,:))
% xlabel('t');
% ylabel('error');
% title('Error en el eje y');
% grid on 
% hold off
% 
% figure 
% subplot(1,2,1)
% hold on 
% plot(t,v_hist(1,:))
% plot(t,v_hist(2,:))
% plot(t,v_hist(3,:))
% plot(t,v_hist(4,:))
% xlabel('t');
% ylabel('v');
% title('Ley de control v');
% grid on
% hold off
% subplot(1,2,2)
% hold on
% plot(t,w_hist(1,:))
% plot(t,w_hist(2,:))
% plot(t,w_hist(3,:))
% plot(t,w_hist(4,:))
% xlabel('t');
% ylabel('w');
% title('Ley de control w');
% grid on 
% hold off

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
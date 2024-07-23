clc
clearvars
close all

%Parametros iniciales
d=0.5;        %Distancia del robot al punto de control
l=0.5;      %Distancia del punto de control al agente virtual
agentes=4;  %Numero de agentes
sigma=0.2;  %Distancia de seguridad
k1=.1;     %Ganancia de evasion
k2=.8;     %Ganancia de consenso
delta=.1;

%Matriz laplaciana (Completamente conectada)
L=[2 -1  0 -1;
  -1  2 -1  0;
   0 -1  2 -1;
  -1  0 -1  2];
I2=eye(2);
Jr=I2;
kronL=kron(L,I2);

%Posiciones iniciales robots reales, elegidas al azar
x=[1 2 3 2];
y=[1 1 -1 -2];
Theta=deg2rad([0 0 90 180]);
%Posiciones iniciales de los puntos de control
alpha1=[x(1)+d*round(cos(Theta(1))); y(1)+d*round(sin(Theta(1)))];
alpha2=[x(2)+d*round(cos(Theta(2))); y(2)+d*round(sin(Theta(2)))];
alpha3=[x(3)+d*round(cos(Theta(3))); y(3)+d*round(sin(Theta(3)))];
alpha4=[x(4)+d*round(cos(Theta(4))); y(4)+d*round(sin(Theta(4)))];
alphaInit=[alpha1;alpha2;alpha3;alpha4];

%Formacion especificada (un rombo)
z1=l*[round(cos((3*pi)/2)); round(sin((3*pi)/2))];
z2=l*[round(cos(2*pi)); round(sin(2*pi))];
z3=l*[round(cos(pi/2)); round(sin(pi/2))];
z4=l*[round(cos(3*pi)); round(sin(pi))];
z=[z1;z2;z3;z4];

%Posiciones iniciales agentes virtuales
alphav1=alpha1+z1;
alphav2=alpha2+z2;
alphav3=alpha3+z3;
alphav4=alpha4+z4;
alphavInit=[alphav1;alphav2;alphav3;alphav4];

%Condiciones de la simulacion
Dt = 0.01; % Periodo de muestreo
tiempo = 10; % Duracion de la simulacion en segundos
iteraciones = tiempo / Dt;

%Inicialización de parametros
alphav=zeros(agentes*2,iteraciones+1); %Agentes virtuales
alphav(:,1)=alphavInit;
alpha=zeros(agentes*2,iteraciones);    %Puntos de control
alpha(:,1)=alphaInit;
theta=zeros(agentes,iteraciones);
theta(:,1)=Theta;
e=zeros(agentes*2,iteraciones);
v_hist=zeros(agentes,iteraciones);
w_hist=zeros(agentes,iteraciones);
contador=0;
e0=0;
%Simulacion
for k=1:iteraciones
    
    for i=1:agentes
        Id=(2*i)-1:(2*i);
        
        %Dinamica
        M = [cos(theta(i,k)) -d*sin(theta(i,k));
             sin(theta(i,k))  d*cos(theta(i,k))];
        %Calculo las posiciones del punto de control 
        alpha(:,k)=alphav(:,k)-z;
        %Posicion del agente en curso
        alphai=alpha(Id(1):Id(2),k);

        %Checar diferencias de posicion por cada agente
        for j=1:agentes
            jId=(2*j)-1:(2*j);
            if i ~= j
               dif_alpha=alphai-alpha(jId(1):jId(2),k);
               if norm(dif_alpha)-sigma<0
                   e0=norm(dif_alpha)-sigma;
                   alpha0=alpha(jId(1):jId(2),k);
               end
            end
        end
        
        if e0<0
            dif_alpha0=alphai-alpha0;
            J0=dif_alpha0'/norm(dif_alpha0);
            e0_dot=k1*e0;
            J0_mas=J0'/(J0*J0');
            No=I2-J0_mas*J0;
            if 0 < det(No)-0.001
                No_mas=inv(No);
            elseif 0 < det(No+delta*eye(2))-0.0001
                No_mas=inv(No+delta*eye(2));
            else
               No_mas=eye(2); 
            end
            ef_dot=-k2*kronL*alphav(:,k);                  
            ut=J0_mas*e0_dot+No_mas*(ef_dot(Id(1):Id(2))-Jr*J0_mas*e0_dot);
            uAplicada=M\ut;
            contador=contador+1;
        else
            ui=-k2*kronL*alphav(:,k);
            uAplicada=M\ui(Id(1):Id(2),:);
        end
        
        %Separo las leyes de control
        v=uAplicada(1);
        w=uAplicada(2);

        %Apoximación de Euler del sistema
        alphav(Id(1):Id(2),k+1)=alphav(Id(1):Id(2),k)+Dt*(M*uAplicada);
        theta(i,k+1)=theta(i,k)+Dt*w;

        %Guardo los errores
        e(:,k)=-kronL*alphav(:,k);
        v_hist(i,k)=v;
        w_hist(i,k)=w;
    end
end

figure
hold on
plot(alpha(1,:),alpha(2,:))
quiver(alpha(1, 1), alpha(2, 1), cos(theta(1,1)), sin(theta(3,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(alpha(1, end), alpha(2, end), cos(theta(1,end)), sin(theta(3,end)), 0.1, 'g', 'LineWidth', .1, 'MaxHeadSize', 2);
plot(alpha(3,:),alpha(4,:))
quiver(alpha(3, 1), alpha(4, 1), cos(theta(2,1)), sin(theta(2,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(alpha(3, end), alpha(4, end), cos(theta(2,end)), sin(theta(2,end)), 0.1, 'g', 'LineWidth', .1, 'MaxHeadSize', 2);
plot(alpha(5,:),alpha(6,:))
quiver(alpha(5, 1), alpha(6, 1), cos(theta(3,1)), sin(theta(3,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(alpha(5, end), alpha(6, end), cos(theta(3,end)), sin(theta(3,end)), 0.1, 'g', 'LineWidth', .1, 'MaxHeadSize', 2);
plot(alpha(7,:),alpha(8,:))
quiver(alpha(7, 1), alpha(8, 1), cos(theta(4,1)), sin(theta(4,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(alpha(7, end), alpha(8, end), cos(theta(4,end)), sin(theta(4,end)), 0.1, 'g', 'LineWidth', .1, 'MaxHeadSize', 2);
xlabel('x');
ylabel('y');
title('Trayectoria de los robots no holónomos');
grid on
hold off

figure
hold on
plot(alphav(1,:),alphav(2,:))
plot(alphav(3,:),alphav(4,:))
plot(alphav(5,:),alphav(6,:))
plot(alphav(7,:),alphav(8,:))
grid on
hold off

t = linspace(0, tiempo, iteraciones);
figure 
subplot(1,2,1)
hold on 
plot(t,e(1,:))
plot(t,e(3,:))
plot(t,e(5,:))
plot(t,e(7,:))
xlabel('t');
ylabel('error');
title('Error en el eje x');
grid on
hold off
subplot(1,2,2)
hold on
plot(t,e(2,:))
plot(t,e(4,:))
plot(t,e(6,:))
plot(t,e(8,:))
xlabel('t');
ylabel('error');
title('Error en el eje y');
grid on 
hold off

figure 
subplot(1,2,1)
hold on 
plot(t,v_hist(1,:))
plot(t,v_hist(2,:))
plot(t,v_hist(3,:))
plot(t,v_hist(4,:))
xlabel('t');
ylabel('v');
title('Ley de control v');
grid on
hold off
subplot(1,2,2)
hold on
plot(t,w_hist(1,:))
plot(t,w_hist(2,:))
plot(t,w_hist(3,:))
plot(t,w_hist(4,:))
xlabel('t');
ylabel('w');
title('Ley de control w');
grid on 
hold off


%% ESTA ES UNA ANIMACION PARA VER LOS CRUCES SI HUBIERA
% num_steps = size(alpha, 2) / 2;  % Dividido por 2 porque cada robot tiene dos filas (x, y)
% figure;
% hold on;
% grid on;
% 
% for t = 1:num_steps
%     clf;  % Clear current figure
%     hold on;
%     plot(alpha(1,1:t), alpha(2,1:t), 'r');  % Trayectoria del robot 1
%     plot(alpha(3,1:t), alpha(4,1:t), 'g');  % Trayectoria del robot 2
%     plot(alpha(5,1:t), alpha(6,1:t), 'b');  % Trayectoria del robot 3
%     plot(alpha(7,1:t), alpha(8,1:t), 'm');  % Trayectoria del robot 4
%     
%     % Dibujar la posición actual de cada robot
%     plot(alpha(1,t), alpha(2,t), 'ro', 'MarkerFaceColor', 'r');
%     plot(alpha(3,t), alpha(4,t), 'go', 'MarkerFaceColor', 'g');
%     plot(alpha(5,t), alpha(6,t), 'bo', 'MarkerFaceColor', 'b');
%     plot(alpha(7,t), alpha(8,t), 'mo', 'MarkerFaceColor', 'm');
%     
%     pause(0.1);  % Pausa de 0.1 segundos para visualizar la animación
% end
% 
% hold off;


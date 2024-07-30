clc
close all
clearvars 

% Coordenadas deseadas
p1 = [0; 0];
p2 = [1; 0];
p3 = [1; -1];
p4 = [0; -1];

% Definir las diferencias cuadradas normadas d^2
d2 = [(p1 - p2)'*(p1 - p2);
      (p1 - p4)'*(p1 - p4);
      (p2 - p3)'*(p2 - p3);
      (p4 - p3)'*(p4 - p3);
      (p1 - p3)'*(p1 - p3);
      (p2 - p4)'*(p2 - p4)];
d2_2 =[(p1 - p2)'*(p1 - p2);
      (p1 - p4)'*(p1 - p4);
      (p2 - p3)'*(p2 - p3);
      (p4 - p3)'*(p4 - p3);
      (p1 - p3)'*(p1 - p3)];
d2_3 =[(p1 - p2)'*(p1 - p2);
      (p1 - p4)'*(p1 - p4);
      (p2 - p3)'*(p2 - p3);
      (p4 - p3)'*(p4 - p3)];
%producto kronecker
I=eye(2);

% Periodo de muestreo
Dt = 0.01;
tiempo = 5; % segundos
iteraciones = tiempo / Dt;

% Posiciones iniciales de los nodos
pInit = [3;2;1;2;-3;-2;3;-2];

%Inicializo el valor de p
p = zeros(8,iteraciones); 
p(:,1)=pInit;
p2 = zeros(8,iteraciones); 
p2(:,1)=pInit;
p3 = zeros(8,iteraciones); 
p3(:,1)=pInit;
sigma=zeros(6,iteraciones);
sigma_2=zeros(5,iteraciones);
sigma_3=zeros(4,iteraciones);

% Simulación con aproximación de Euler
for k = 1:iteraciones
    %% RIGIDA
    e1=p(3:4,k)-p(1:2,k);%e1=p2-p1
    e2=p(7:8,k)-p(1:2,k);%e2=p4-p1
    e3=p(5:6,k)-p(3:4,k);%e3=p3-p2
    e4=p(5:6,k)-p(7:8,k);%e4=p3-p4
    e5=p(5:6,k)-p(1:2,k);%e5=p3-p1
    e6=p(7:8,k)-p(3:4,k);%e6=p4-p2
    e=blkdiag(e1',e2',e3',e4',e5',e6');
    %Calculo los valores de sigma
    sigma1=e1'*e1-d2(1);
    sigma2=e2'*e2-d2(2);
    sigma3=e3'*e3-d2(3);
    sigma4=e4'*e4-d2(4);
    sigma5=e5'*e5-d2(5);
    sigma6=e6'*e6-d2(6);
    sigma(:,k)=[sigma1;sigma2;sigma3;sigma4;sigma5;sigma6];
    %Matrices de incidencia locales estructura rigida
    E = [1 -1  0  0;
         1  0  0 -1;
         0  1 -1  0;
         0 -1  0  1;
         1  0 -1  0;
         0  1  0 -1];
    kronp=kron(E,I);
    %Calculo del vector R
    R=e*kronp;
    
    % Calcular la derivada de p
    u = -R' * R * p(:,k) - R' * d2;
    
    % Aproximación de Euler
    p(:,k+1) = p(:,k) + Dt * u;
    
    %% MINIMAMENTE RIGIDA

    e1_2=p2(3:4,k)-p2(1:2,k);%e1=p2-p1
    e2_2=p2(7:8,k)-p2(1:2,k);%e2=p4-p1
    e3_2=p2(5:6,k)-p2(3:4,k);%e3=p3-p2
    e4_2=p2(5:6,k)-p2(7:8,k);%e4=p3-p4
    e5_2=p2(5:6,k)-p2(1:2,k);%e5=p3-p1
    e2=blkdiag(e1_2',e2_2',e3_2',e4_2',e5_2');
    %Calculo los valores de sigma
    sigma1_2=e1_2'*e1_2-d2(1);
    sigma2_2=e2_2'*e2_2-d2(2);
    sigma3_2=e3_2'*e3_2-d2(3);
    sigma4_2=e4_2'*e4_2-d2(4);
    sigma5_2=e5_2'*e5_2-d2(5);
    
    sigma_2(:,k)=[sigma1_2;sigma2_2;sigma3_2;sigma4_2;sigma5_2];
    %Matrices de incidencia locales cosa rigida
        E2 = [1 -1  0  0;
              1  0  0 -1;
              0  1  0 -1;
              0  0 -1  1;
              1  0 -1  0];
        kronp2=kron(E2,I);
    %Calculo del vector R
        R2=e2*kronp2;
    % Calcular la derivada de p
    u2 = -R2' * R2 * p2(:,k) - R2' * d2_2;
    
    % Aproximación de Euler
    p2(:,k+1) = p2(:,k) + Dt * u2;
    
    %% NO RIGIDA
    e1_3=p3(3:4,k)-p3(1:2,k);%e1=p2-p1
    e2_3=p3(7:8,k)-p3(1:2,k);%e2=p4-p1
    e3_3=p3(5:6,k)-p3(3:4,k);%e3=p3-p2
    e4_3=p3(5:6,k)-p3(7:8,k);%e4=p3-p4
    e_3=blkdiag(e1_2',e2_2',e3_2',e4_2');
     %Calculo los valores de sigma
    sigma1_3=e1_3'*e1_3-d2(1);
    sigma2_3=e2_3'*e2_3-d2(2);
    sigma3_3=e3_3'*e3_3-d2(3);
    sigma4_3=e4_3'*e4_3-d2(4);
    
    sigma_3(:,k)=[sigma1_3;sigma2_3;sigma3_3;sigma4_3];
    %Matrices de incidencia locales cosa rigida
        E3 = [1 -1  0  0;
              1  0  0 -1;
              0  1 -1  0;
              0  0 -1  1];
        kronp3=kron(E3,I);
    %Calculo del vector R
        R3=e_3*kronp3;
    % Calcular la derivada de p
    u3 = -R3' * R3 * p3(:,k) - R3' * d2_3;
    
    % Aproximación de Euler
    p3(:,k+1) = p3(:,k) + Dt * u3;
end

figure
hold on
plot(p(1,:),p(2,:))
plot(p(3,:),p(4,:))
plot(p(5,:),p(6,:))
plot(p(7,:),p(8,:))
%Lineas entre nodos
plot([p(1, end), p(3, end)], [p(2, end), p(4, end)], 'k--');
plot([p(1, end), p(7, end)], [p(2, end), p(8, end)], 'k--');
plot([p(3, end), p(5, end)], [p(4, end), p(6, end)], 'k--');
plot([p(7, end), p(5, end)], [p(8, end), p(6, end)], 'k--');
plot([p(1, end), p(5, end)], [p(2, end), p(6, end)], 'k--');
plot([p(3, end), p(7, end)], [p(4, end), p(8, end)], 'k--');
title('Evolucion posiciones estructura rigida')
xlabel('x')
ylabel('y')
axis equal
grid on
hold off

figure
hold on
plot(p2(1,:),p2(2,:))
plot(p2(3,:),p2(4,:))
plot(p2(5,:),p2(6,:))
plot(p2(7,:),p2(8,:))
%Lineas entre nodos
plot([p2(1, end), p2(3, end)], [p2(2, end), p2(4, end)], 'k--');
plot([p2(1, end), p2(7, end)], [p2(2, end), p2(8, end)], 'k--');
plot([p2(3, end), p2(5, end)], [p2(4, end), p2(6, end)], 'k--');
plot([p2(7, end), p2(5, end)], [p2(8, end), p2(6, end)], 'k--');
plot([p2(1, end), p2(5, end)], [p2(2, end), p2(6, end)], 'k--');
title('Evolucion posiciones estructura minimamente rigida')
xlabel('x')
ylabel('y')
axis equal
grid on
hold off

figure
hold on
plot(p3(1,:),p3(2,:))
plot(p3(3,:),p3(4,:))
plot(p3(5,:),p3(6,:))
plot(p3(7,:),p3(8,:))
%Lineas entre nodos
plot([p3(1, end), p3(3, end)], [p3(2, end), p3(4, end)], 'k--');
plot([p3(1, end), p3(7, end)], [p3(2, end), p3(8, end)], 'k--');
plot([p3(3, end), p3(5, end)], [p3(4, end), p3(6, end)], 'k--');
plot([p3(7, end), p3(5, end)], [p3(8, end), p3(6, end)], 'k--');
title('Evolucion posiciones estructura no rigida')
xlabel('x')
ylabel('y')
axis equal
grid on
hold off

t = linspace(0, tiempo, iteraciones);
figure
plot(t,sigma)
grid on

figure
plot(t,sigma_2)
grid on

figure
plot(t,sigma_3)
grid on

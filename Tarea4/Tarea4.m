clc
clearvars 
close all 

%Matriz de grado
%Primer punto
D1=[1 0 0 0 0;
    0 2 0 0 0;
    0 0 1 0 0;
    0 0 0 1 0;
    0 0 0 0 0];
D2=[2 0 0 0 0;
    0 3 0 0 0;
    0 0 2 0 0;
    0 0 0 2 0;
    0 0 0 0 0];
D3=[3 0 0 0 0;
    0 1 0 0 0;
    0 0 2 0 0;
    0 0 0 1 0;
    0 0 0 0 0];
D4=[1 0 0 0 0;
    0 2 0 0 0;
    0 0 1 0 0;
    0 0 0 2 0;
    0 0 0 0 0];



%Matriz de adyacencia
%Primer punto
A1=[0 0 0 0 1;
    1 0 1 0 0;
    0 1 0 0 0;
    1 0 0 0 0;
    0 0 0 0 0];
A2=[0 0 1 0 1;
    1 0 1 0 1;
    0 1 0 0 1;
    1 0 0 0 1;
    0 0 0 0 0];
A3=[0 1 1 1 0;
    0 0 1 0 0;
    0 1 0 0 1;
    0 0 0 0 1;
    0 0 0 0 0];
A4=[0 0 1 0 0;
    1 0 1 0 0;
    0 1 0 0 0;
    1 0 0 0 1;
    0 0 0 0 0];



%hago el objeto grafo, pero aún no lo ploteo
G1=digraph(A1');
G2=digraph(A2');
G3=digraph(A3');
G4=digraph(A4');

%Nodos
n=size(A1,1);
% 
%Laplaciano
L1=D1-A1
L2=D2-A2
L3=D3-A3
L4=D4-A4
% 
%Calculo de los eigenvalores
eig(-L1)
eig(-L2)
eig(-L3)
eig(-L4)
% 
% %Formo la matríz Gama
% n=size(L,1);
% l=3; %Posición, velocidad, aceleración
% gammas=[2 1 3];%defino mis valores de gamma
% 
% Gamma=zeros(l*n,l*n); %Inicializo la matriz Gamma
% I=eye(n);
% 
% %relleno la matriz Gamma
% for i = 1:l-1
%     Gamma((i-1)*n+1:i*n, i*n+1:(i+1)*n) = I;
% end
% 
% % Rellenar la última fila de bloques de Gamma
% Gamma((l-1)*n+1:l*n, :) = [-gammas(1)*L, -gammas(2)*L, -gammas(3)*L];
% 
% 
%Hago el producto Kronecker
m=1; %dimensiones que estoy considerando
Im=eye(m);  
Gamma_kron= kron(L1,Im);
% 
% %% Esta parte es para la simulación del sistema
% %Valores iniciales
k=0;
% ref=cos(k); %La referencia que quiero que sigan
xi0=[1.5;.5;0;-0.5;cos(k)];%Guardo los valores iniciales para referencia
xi=xi0;%Uso estos valores en la aproximación de Euler
% 
%periodo de muestreo
Dt=0.01;
tiempo=25; %segundos
iteraciones=tiempo/Dt;

%Simulo el sistema
for k=1:iteraciones
    %Aproximación de Euler con Gamma_kron
    xi(:,k+1)=xi(:,k)-Dt*(Gamma_kron*xi(:,k));
end

%Calculo de consensos
% syms t;
% Uno=ones(n,1);
% Cero=zeros(n,n);
% p=[1;0;0;0];
% egamma=[Uno*p' t*Uno*p' (1/2)*(t^2)*Uno*p';
%         Cero   Uno*p'   t*Uno*p';
%         Cero   Cero     Uno*p'];
% Consenso=egamma*xi0

%Vectores de tiempo para gráficar
t=linspace(0,tiempo,iteraciones+1);
% te=linspace(0,tiempo,iteraciones);
% 
% figure
% subplot(1,2,1)
% plot(t,xi(1:4,:))
% grid on
% 
% subplot(1,2,2)
% plot(t,xi(5:8,:))
% grid on
% 
% 
figure
plot(t,xi)
grid on

figure
subplot(2,2,1)
plot(G1);
title('inciso a')
subplot(2,2,2)
plot(G2);
title('inciso b')
subplot(2,2,3)
plot(G3);
title('inciso c')
subplot(2,2,4)
plot(G4);
title('inciso d')




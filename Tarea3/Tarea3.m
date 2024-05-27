clc
clearvars 
close all 

%Matriz de grado
%Primer punto
% D=[0 0 0 0;
%    0 1 0 0;
%    0 0 1 0;
%    0 0 0 1;];
%Segundo punto
D=[1 0 0 0;
   0 2 0 0;
   0 0 2 0;
   0 0 0 1;];


%Matriz de adyacencia
%Primer punto
% A=[0 0 0 0;
%    1 0 0 0;
%    0 1 0 0;
%    0 0 1 0;];
%Segundo punto
A=[0 1 0 0;
   1 0 1 0;
   0 1 0 1;
   0 0 1 0;];


%hago el objeto grafo, pero aún no lo ploteo
% G=digraph(A');
G=graph(A);

%Nodos
n=size(A,1);

%Laplaciano
L=D-A

%SVD DEL LAPLACIANO(es mejor calcularlo en wolfram)
[U,D,V]=svd(L);

%Formo la matríz Gama
n=size(L,1);
l=3; %Posición, velocidad, aceleración
gammas=[1 2 3];%defino mis valores de gamma

Gamma=zeros(l*n,l*n); %Inicializo la matriz Gamma
I=eye(n);

%relleno la matriz Gamma
for i = 1:l-1
    Gamma((i-1)*n+1:i*n, i*n+1:(i+1)*n) = I;
end

% Rellenar la última fila de bloques de Gamma
Gamma((l-1)*n+1:l*n, :) = [-gammas(1)*L, -gammas(2)*L, -gammas(3)*L];


%Hago el producto Kronecker
m=1; %dimensiones que estoy considerando
Im=eye(m);  
Gamma_kron= kron(Gamma,Im);

%% Esta parte es para la simulación del sistema
%Valores iniciales
xi0=[1;2;3;4;5;6;7;8;9;10;11;12];%Guardo los valores iniciales para referencia
% xi0=rand(size(Gamma_kron,1),1);%Guardo los valores iniciales para referencia
xi=xi0;%Uso estos valores en la aproximación de Euler

%periodo de muestreo
Dt=0.01;
tiempo=25; %segundos
iteraciones=tiempo/Dt;

%Simulo el sistema
for k=1:iteraciones

    %Aproximación de Euler con Gamma_kron
    xi(:,k+1)=xi(:,k)+Dt*(Gamma_kron*xi(:,k));

 
end

%Calculo de consensos
syms t;
Uno=ones(n,1);
Cero=zeros(n,n);
p=[1;0;0;0];
egamma=[Uno*p' t*Uno*p' (1/2)*(t^2)*Uno*p';
        Cero   Uno*p'   t*Uno*p';
        Cero   Cero     Uno*p'];
Consenso=egamma*xi0

%Vectores de tiempo para gráficar
t=linspace(0,tiempo,iteraciones+1);
te=linspace(0,tiempo,iteraciones);

figure
subplot(1,2,1)
plot(t,xi(1:4,:))
grid on

subplot(1,2,2)
plot(t,xi(5:8,:))
grid on


figure
plot(t,xi(9:12,:))
grid on

figure
plot(G, 'Layout', 'force', 'EdgeLabel', G.Edges.Weight);



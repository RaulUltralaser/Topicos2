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

%Caso especial inciso d)
L=[3 -1 -1 -1 0;
   -1 2 -1 0 0;
   -1 -1 2 0 0;
   -1 0 0 2 -1;
   0 0 0 0 0];
%Caso especial inciso f)
Lf=[1 0 0 0 -1;
    0 1 0 0 -1;
    0 0 1 0 -1;
    0 0 0 1 -1;
    0 0 0 0 0];

%Calculo de los eigenvalores
e1=eig(-L1);
e2=eig(-L2);
e3=eig(-L3);
e4=eig(-L4);
svd(-L1)
svd(-L2)
svd(-L3)
svd(-L4)
% 
%Hago el producto Kronecker
m=1; %dimensiones que estoy considerando
Im=eye(m);  
Kron=kron(L,Im);   %Este es para el inciso d)
Kronf=kron(Lf,Im); %Este es para el inciso f)
Kron1= kron(L1,Im);
Kron2= kron(L2,Im);
Kron3= kron(L3,Im);
Kron4= kron(L4,Im);
% 
% %% Esta parte es para la simulación del sistema

% %Valores iniciales
ref=1; %La referencia que quiero que sigan
xi0=[-0.5;0;.5;1.5;ref];%Guardo los valores iniciales para referencia
xi=xi0;xif=xi0;xi1=xi0;xi2=xi0;xi3=xi0;xi4=xi0;xi5=xi0;%Uso estos valores en la aproximación de Euler
% 
%periodo de muestreo
Dt=0.01;
tiempo=20; %segundos
iteraciones=tiempo/Dt;

%Simulo el sistema
for k=1:iteraciones
    %Aproximación de Euler con Gamma_kron
    xi(:,k+1)=xi(:,k)-Dt*(Kron*xi(:,k));     %Inciso d)
    xif(:,k+1)=xif(:,k)-Dt*(Kronf*xif(:,k)); %Inciso f)
    xi1(:,k+1)=xi1(:,k)-Dt*(Kron1*xi1(:,k));
    xi2(:,k+1)=xi2(:,k)-Dt*(Kron2*xi2(:,k));
    xi3(:,k+1)=xi3(:,k)-Dt*(Kron3*xi3(:,k));
    xi4(:,k+1)=xi4(:,k)-Dt*(Kron4*xi4(:,k));
end


%Vectores de tiempo para gráficar
t=linspace(0,tiempo,iteraciones+1);

figure
subplot(2,2,1)
plot(t,xi1);
title('inciso a')
subplot(2,2,2)
plot(t,xi2);
title('inciso b')
subplot(2,2,3)
plot(t,xi3);
title('inciso c')
subplot(2,2,4)
plot(t,xi4);
title('inciso d')

figure
plot(t,xi);
title('inciso d)')
xlabel('Tiempo s')
ylabel('Estados')

figure
plot(t,xif);
title('inciso f)')
xlabel('Tiempo s')
ylabel('Estados')

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






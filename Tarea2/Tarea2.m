clc
clearvars 
close all 

%Matriz de grado
%Primer punto
% D=[0 0 0 0 0;
%     0 5 0 0 0;
%     0 0 5 0 0;
%     0 0 0 6 0;
%     0 0 0 0 10];
%Segundo Punto
D=[0 0 0 0 0;
    0 5 0 0 0;
    0 0 5 0 0;
    0 0 0 9 0;
    0 0 0 0 13];

%Matriz de adyacencia
% A=[0 0 0 0 0;
%    3 0 2 0 0;
%    0 0 0 5 0;
%    6 0 0 0 0;
%    0 2 0 8 0];
%Segundo Punto
A=[0 0 0 0 0;
   3 0 2 0 0;
   0 0 0 5 0;
   6 1 2 0 0;
   0 2 3 8 0];

%hago el objeto grafo, pero aún no lo ploteo
G=digraph(A');

%Nodos
n=size(A,1);

%Laplaciano
L=D-A

%SVD DEL LAPLACIANO(es mejor calcularlo en wolfram)
[U,D,V]=svd(L);

%Valores iniciales
x=[1;2;3;4;5];
% x=10*rand(6,1); %errores
xl=x; %laplaciano

%Calculo error en estado estable
% SS=(1/n)*U(:,n)*U(:,n)'*x;

%Inicializo e
e=[0,0,0,0,0];

%periodo de muestreo
Dt=0.01;
tiempo=7; %segundos
iteraciones=tiempo/Dt;

for k=1:iteraciones
    %función que calcula los errores entre estados
    e=calcular_e(A,x,k,n);
    %aproximación de Euler de los estados
    x(:,k+1)=x(:,k)+Dt*(e');

    %Calcula errores entre estados con laplaciano
    eL=calcular_e(A,xl,k,n);
    %Aproximación de Euler con Laplaciano
    xl(:,k+1)=xl(:,k)+Dt*(-L*xl(:,k));

    %Almacena errores para poder graficarlos
    etotal(:,k)=e;
    eLtotal(:,k)=eL;
end

%Vectores de tiempo para gráficar
t=linspace(0,tiempo,iteraciones+1);
te=linspace(0,tiempo,iteraciones);

figure
subplot(1,2,1)
plot(t,x);
ylim([0,6])
title('Evolución de estados', 'con errores')
grid on
subplot(1,2,2)
plot(te,etotal)
title('Error de consenso')
grid on

figure
subplot(1,2,1)
plot(t,xl);
ylim([0,6])
title('Evolución de estados', 'con Laplaciano')
grid on
subplot(1,2,2)
plot(te,eLtotal)
title('Error de consenso')
grid on

figure
plot(G, 'Layout', 'force', 'EdgeLabel', G.Edges.Weight);

function e=calcular_e(A,x,iteracion,n)
    for i = 1:n
        suma = 0;
            for j = 1:n
                % Verificar si a_ij es 1 usando la matriz de adyacencia
                if A(i, j) >= 1
                    suma = suma+A(i,j)*(x(j,iteracion) - x(i,iteracion));
                end
            end
        % Asignar el resultado al vector de errores e en la fila i
        e(i) = suma;
    end
end
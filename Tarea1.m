clc
clearvars 
close all 

%Matriz de grado
 D=[2 0 0 0 0 0;
    0 2 0 0 0 0;
    0 0 2 0 0 0;
    0 0 0 2 0 0;
    0 0 0 0 2 0;
    0 0 0 0 0 2];

%Matriz de adyacencia
 A=[0 1 0 0 0 1;
    1 0 1 0 0 0;
    0 1 0 1 0 0;
    0 0 1 0 1 0;
    0 0 0 1 0 1;
    1 0 0 0 1 0];

%hago el objeto grafo, pero aún no lo ploteo
G=graph(A);

%Laplaciano
L=D-A;

%Valores iniciales
% x=[1;2;3;4;5;6];
x=10*rand(6,1); %errores
xl=x; %laplaciano

%Inicializo e
e=[0,0,0,0,0,0];

%periodo de muestreo
Dt=0.01;
tiempo=10; %segundos
iteraciones=tiempo/Dt;

for k=1:iteraciones
    %función que calcula los errores entre estados
    e=calcular_e(A,x,k);
    %aproximación de Euler de los estados
    x(:,k+1)=x(:,k)+Dt*(e');

    %Calcula errores entre estados con laplaciano
    eL=calcular_e(A,xl,k);
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
title('Evolución de estados', 'con errores')
grid on
subplot(1,2,2)
plot(te,etotal)
title('Error de consenso')
grid on

figure
subplot(1,2,1)
plot(t,xl);
title('Evolución de estados', 'con Laplaciano')
grid on
subplot(1,2,2)
plot(te,eLtotal)
title('Error de consenso')
grid on

figure
plot(G);

function e=calcular_e(A,x,iteracion)
    for i = 1:6
        suma = 0;
            for j = 1:6
                % Verificar si a_ij es 1 usando la matriz de adyacencia
                if A(i, j) == 1
                    suma = suma+(x(j,iteracion) - x(i,iteracion));
                end
            end
        % Asignar el resultado al vector de errores e en la fila i
        e(i) = suma;
    end
end
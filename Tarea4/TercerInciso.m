clc
clearvars 
close all 

%Laplacianos
L1=[1 0 -1 0 0;
    0 0 0 0 0;
    -1 0 1 0 0;
    0 0 0 0 0;
    0 0 0 0 0];

L2=[1 -1 0 0 0;
    -1 1 0 0 0;
    0 0 0 0 0;
    0 0 0 0 0;
    0 0 0 0 0];

L3=[1 0 -1 0 0;
    0  0 0 0 0;
    -1 0 1 0 0;
    0  0 0 0 0;
    0  0 0 0 0];

L4=[0 0 0 0 0;
    0 0 0 0 0;
    0 0 1 -1 0;
    0 0 -1 1 0;
    0 0 0 0 0];

L5=[0 0 0 0 0;
    0 0 0 0 0;
    0 0 0 0 0;
    0 0 0 1 -1;
    0 0 0 -1 1];

% 
%Hago el producto Kronecker
m=1; %dimensiones que estoy considerando
Im=eye(m);  
Kron1= kron(L1,Im);
Kron2= kron(L2,Im);
Kron3= kron(L3,Im);
Kron4= kron(L4,Im);
Kron5= kron(L5,Im);
% 
% %% Esta parte es para la simulación del sistema

% %Valores iniciales
xi0=[.2;0.4;0.6;0.8;1];%Guardo los valores iniciales para referencia
xi=xi0;%Uso estos valores en la aproximación de Euler
% 
%periodo de muestreo
Dt=0.01;
tiempo=250; %segundos
iteraciones=tiempo/Dt;

%Simulo el sistema
for k=1:iteraciones
     % Selección aleatoria de la topología cada dos segundos
    if mod(k, 200) == 1
        rand_index = randi([1, 5]);
    end
    
    % Selección del producto de Kronecker correspondiente
    switch rand_index
        case 1
            Kron = Kron1;
        case 2
            Kron = Kron2;
        case 3
            Kron = Kron3;
        case 4
            Kron = Kron4;
        case 5
            Kron = Kron5;
    end
    %Aproximación de Euler con Gamma_kron
    xi(:,k+1)=xi(:,k)-Dt*(Kron*xi(:,k));    
end


%Vectores de tiempo para gráficar
t=linspace(0,tiempo,iteraciones+1);


figure
plot(t,xi);
xlabel('Tiempo');
ylabel('Estado');
title('Consenso con topología cambiante')






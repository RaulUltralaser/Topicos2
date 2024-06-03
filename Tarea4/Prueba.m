clc
close all

L=[1 0 0 0;
  -1 2 -1 0;
  0 -1 1 0;
  -1 0 0 1];

alphas=[2 2 1 1];
Gamma=diag(alphas);

m=1; %dimensiones que estoy considerando
Im=eye(m); 
Gamma_kron=kron(L+Gamma,Im);

xi0=[1.5;.5;0;-0.5];%Guardo los valores iniciales para referencia
xi=xi0;%Uso estos valores en la aproximación de Euler
% 
%periodo de muestreo
Dt=0.01;
tiempo=25; %segundos
iteraciones=tiempo/Dt;


% xi = zeros(num_agents, iteraciones); % Estados de los agentes
r = cos(0:Dt:(iteraciones-1)*Dt); % Ref

%Simulo el sistema
for k=1:iteraciones
    ref = r(k) * ones(4, 1);
    %Aproximación de Euler con Gamma_kron
    xi(:,k+1)=xi(:,k)-Dt*(Gamma_kron*(xi(:,k)-ref));
end

t=linspace(0,tiempo,iteraciones+1);
time=linspace(0,tiempo,iteraciones);
figure
plot(t,xi)
hold on
plot(time, r, '--', 'LineWidth', 2); % Referencia
grid on
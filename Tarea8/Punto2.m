clearvars 
close all
clc

% Parámetros iniciales
l = 0.5; % Distancia del centro del robot al punto de control
% Ganancias del controlador
k1=1;
k2=1;
k3=1; 
k4=2;
k0=2;
num_agentes = 4; %Numero de agentes
Rang=0.1;        %Establece el rango de sensado para todos los agentes
ro=Rang;         % Es el rango del obstáculo
kappa=1;         %Esto es para el feed forward term

%Matriz Laplaciana
L=[1  0  0 -1;
  -1  1  0  0;
   0 -1  1  0;
   0  0 -1  1];
I = eye(3);

%Posiciones inciales
x=[1; 2; 3; 2];
y=[1; 1; -1; -2];
Angulo=[0; 0; 90; 180];

% Datos de la simulación
h = 0.01; % Periodo de muestreo
tiempo = 50; % Duración de la simulación en segundos
iteraciones = round(tiempo / h);

%Parámetros para la función bump
tau = 1;          
t0 = 0;           % Tiempo inicial en el que se quiere que inicie la función
n = tiempo;           % Duración en segundos durante la cual la función debe estar activa
bump=zeros(1,iteraciones);

% Inicializar el ángulo theta y trayectorias de cada agente
theta = zeros(num_agentes, iteraciones + 1);
x_hist = zeros(num_agentes, iteraciones + 1);
y_hist = zeros(num_agentes, iteraciones + 1);
z_hist = zeros(3 * num_agentes, iteraciones + 1);
q_hist = zeros(3 * num_agentes, 1);
alpha_hist=zeros(3 * num_agentes, iteraciones + 1);

Xi1 = zeros(3 * num_agentes, iteraciones + 1);
qid=zeros(3*num_agentes,iteraciones + 1);
qiu=zeros(3*num_agentes,iteraciones + 1);
qic=zeros(3*num_agentes,iteraciones + 1);
ei=zeros(3,iteraciones + 1);
vi=zeros(num_agentes,iteraciones+1);
wi=zeros(num_agentes,iteraciones+1);
dm=zeros(num_agentes,iteraciones+1);
vl=zeros(num_agentes,iteraciones+1);

%Se declaran vaiables auxiliares donde se almacenarán la diferencia de las
%posiciones de los agentes
pi_ox=zeros(num_agentes,num_agentes);
pi_oy=zeros(num_agentes,num_agentes);
norm_pi_o=zeros(num_agentes,num_agentes);
thetaio=zeros(num_agentes,num_agentes);
vthetaio=zeros(num_agentes,num_agentes);
% Se define el parámetro de desplazamiento z
z=1;

%Desplazamientos deseados
 delta= [0; % d41 x
         2; % d41 y 
         0;
        -2; % d12 x
         0; % d12 y
         0;
         0; % d23 x
        -2; % d23 y
         0;
         2; % d34 x
         0; % d34 y
         0];

for p = 1:num_agentes
    theta(p, 1) = deg2rad(Angulo(p, 1));
    x_hist(p, 1) = x(p, 1);
    y_hist(p, 1) = y(p, 1);
    % Organizar las posiciones y ángulos en el vector qid
    qid(3 * p - 2, 1) = x(p, 1) + l * cos(theta(p, 1));       % Posición en x
    qid(3 * p - 1, 1) = y(p, 1) + l * sin(theta(p, 1));       % Posición en y
    qid(3 * p, 1) = theta(p, 1);       % Ángulo theta
    z_hist(3 * p - 2, 1) = x_hist(p, 1) + l * cos(theta(p, 1));
    z_hist(3 * p-1, 1) = y_hist(p, 1) + l * sin(theta(p, 1));
    z_hist(3 * p, 1) = theta(p,1);
    alpha_hist(3*p-2,1)=z_hist(3*p-2,1) + z * cos(theta(p, 1));
    alpha_hist(3 * p-1, 1) = z_hist(3*p-1,1)+z * sin(theta(p, 1));
    alpha_hist(3 * p, 1) = theta(p,1);
end

% Inicializar trayectorias de velocidad
v_hist = zeros(num_agentes, iteraciones);
w_hist = zeros(num_agentes, iteraciones);

% Simulación
for k = 1:iteraciones

    %----- Xi es para calcular la diferencia entre la posición del punto de
    %control alpha de los vecinos y el ángulo theta. Como solo queremos que lleguen a
    %consenso, es un consenso simple considerando la evasión. Ojo el qiu
    %contiene el vector de todoso los agentes, algunos pueden estar
    %evadiendo y otros no. 
    
    Xi1(:, k+1) =-k4*kron(L, I)*(alpha_hist(:, k)+delta);
    %Calcular función bump en cada iteración
    t = k * h;  % Definir t en cada iteración del ciclo como tiempo en segundos
    if t0 <= t && t < n + t0
        bump(k) = exp((-tau) /  (1- ((t - t0) / n)^2));
    else
        bump(k) = 0;
    end
    % Sensado de cada agente con respecto a los demás
    for p = 1:num_agentes
        agentes_cercanos = [];
        distancias_cercanas = [];

        for j = 1:num_agentes
            if p ~= j
                % Calcular la diferencia de posición en x y en y
                pi_ox(p,j) = x_hist(p, k) + l * cos(theta(p, k)) - x_hist(j, k)- l * cos(theta(j, k));
                pi_oy(p,j) = y_hist(p, k)+ l * sin(theta(p, k)) - y_hist(j, k)- l * sin(theta(p, k));
                norm_pi_o(p,j) = sqrt((pi_ox(p,j))^2 + (pi_oy(p,j))^2);
                thetaio(p,j) = atan2(-pi_ox(p,j), -pi_oy(p,j));
                vthetaio(p,j) = abs(theta(p,k) - thetaio(p,j));
                
                % Verificar si el agente j está dentro del rango y ángulo
                if norm_pi_o(p,j) <= dm(p,k) && vthetaio(p,j) < pi/2
                    agentes_cercanos = [agentes_cercanos; j];
                    distancias_cercanas = [distancias_cercanas; norm_pi_o(p,j)];
                end
            end
        end
        % Determinar el valor de gamma y el agente más cercano
        if ~isempty(agentes_cercanos)
            [min_distancia, idx] = min(distancias_cercanas);
            agente_mas_cercano = agentes_cercanos(idx);
            gamma = 1;
            
            % Tomar las coordenadas del agente más cercano
            x_cercano = x_hist(agente_mas_cercano, k)+ l * cos(theta(agente_mas_cercano, k));
            y_cercano = y_hist(agente_mas_cercano, k)+ l * sin(theta(agente_mas_cercano, k));
            
            % Asignar el ángulo theta_io
            thetaio(p, agente_mas_cercano) = atan2(-x_cercano, -y_cercano);
            
            % Asignar el vector p_io
            p_io = [x_cercano; y_cercano];
            
            % Buscar un vector ortogonal a p_io
            p_iorto = [-y_cercano; x_cercano];
        else
            gamma = 0;
            x_cercano = 0; % Valor por defecto si no hay agentes cercanos
            y_cercano = 0; % Valor por defecto si no hay agentes cercanos
            p_io = [0; 0]; % Valor por defecto si no hay agentes cercanos
            p_iorto = [0; 0]; % Valor por defecto si no hay agentes cercanos
        end
        qic(1)=p_iorto(1);
        qic(2)=p_iorto(2);
        thetaic=atan2(p_iorto(2)-y_hist(p,k),p_iorto(1)-x_hist(p,k));
        quic(3)=thetaic;
        % Actualizamos si evadimos o vamos a consenso
        qiu(3 * p - 2, k) = (1 - gamma) * Xi1(3*p-2, k) + gamma * qic(1);
        qiu(3 * p - 1, k) = (1 - gamma) * Xi1(3*p-1, k) + gamma * qic(2);
        qiu(3 * p, k) = (1 - gamma) *Xi1(3*p, k) + gamma * qic(3);
        %----- En cada iteración actualiza la distancia que debe mantener cada
        %agente para su distancia segura.
        if sin(norm(vthetaio(p,:)))==0
        dm(p,k) = 0;
        else
        dm(p,k) = min(Rang, ro / (sin(norm(vthetaio(p,:)))));
        end
        %---Calculamos la matriz de rotación-ángulo
        R = [cos(theta(p, k)) sin(theta(p, k)) 0;
            -sin(theta(p, k))  cos(theta(p, k)) 0;
            0                       0           1];
        %------- Calcula el error en el marco de referencia local con sus
        %vecinos
        ei = R * [qiu(3 * p - 2, k); qiu(3 * p - 1, k);qiu(3 * p, k)];
        
        vl(:,k)=L*vi(:,k);

        if  kappa <= norm(vl(p,k))
            vd(p,k)=k0*exp(-tau);
        else
            vd(p,k)=k0*bump(k);
        end
        
        vi(p,k) = vd(p,k) * cos(ei(3)) + k1 * ei(1);
        
        if ei(3)==0
            wi(p,k) = k2 * vd(p,k) * ei(2) + k3 * ei(3);
        else
        wi(p,k) = k2 * vd(p,k) * ei(2) * (sin(ei(3))) / (ei(3)) + k3 * ei(3);
        end

        % Dinámica del sistema
        x_hist(p, k + 1) = x_hist(p, k) + h * vi(p,k) * cos(theta(p, k));
        y_hist(p, k + 1) = y_hist(p, k) + h * vi(p,k) * sin(theta(p, k));
        theta(p, k + 1) = theta(p, k) + h * wi(p,k);

         % Actualizar el vector z con los nuevos valores de x y y
        z_hist(3 * p - 2, k + 1) = x_hist(p, k + 1) + l * cos(theta(p, k+1));
        z_hist(3 * p-1, k + 1) = y_hist(p, k + 1) + l * sin(theta(p, k+1));
        z_hist(3 * p, k + 1) = theta(p,k+1);
        alpha_hist(3 * p - 2,k+1)=z_hist(3*p-2, k + 1)+ z * cos(theta(p, k+1));
        alpha_hist(3 * p-1,k+1)=z_hist(3*p-1, k + 1)+ z * sin(theta(p, k+1));
        alpha_hist(3 * p, k + 1) = theta(p,k+1);
    end
end

figure
hold on
plot(z_hist(1,:),z_hist(2,:))
quiver(z_hist(1, 1), z_hist(2, 1), cos(theta(1,1)), sin(theta(3,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(z_hist(1, end), z_hist(2, end), cos(theta(1,end)), sin(theta(3,end)), 0.1, 'g', 'LineWidth', .1, 'MaxHeadSize', 2);
plot(z_hist(4,:),z_hist(5,:))
quiver(z_hist(4, 1), z_hist(5, 1), cos(theta(2,1)), sin(theta(2,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(z_hist(4, end), z_hist(5, end), cos(theta(2,end)), sin(theta(2,end)), 0.1, 'g', 'LineWidth', .1, 'MaxHeadSize', 2);
plot(z_hist(7,:),z_hist(8,:))
quiver(z_hist(7, 1), z_hist(8, 1), cos(theta(3,1)), sin(theta(3,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(z_hist(7, end), z_hist(8, end), cos(theta(3,end)), sin(theta(3,end)), 0.1, 'g', 'LineWidth', .1, 'MaxHeadSize', 2);
plot(z_hist(10,:),z_hist(11,:))
quiver(z_hist(10, 1), z_hist(11, 1), cos(theta(4,1)), sin(theta(4,1)), 0.1, 'r', 'LineWidth', .1, 'MaxHeadSize', 1 );
quiver(z_hist(10, end), z_hist(11, end), cos(theta(4,end)), sin(theta(4,end)), 0.1, 'g', 'LineWidth', .1, 'MaxHeadSize', 2);
xlabel('x');
ylabel('y');
title('Trayectoria de los robots no holónomos');
grid on
hold off

t = linspace(0, tiempo, iteraciones);
figure 
subplot(1,2,1)
hold on 
plot(t,qid(1, 1:iteraciones ) - z_hist(2, 1:iteraciones ))
plot(t,qid(4, 1:iteraciones ) - z_hist(5, 1:iteraciones ))
plot(t,qid(7, 1:iteraciones ) - z_hist(8, 1:iteraciones ))
plot(t,qid(10, 1:iteraciones ) - z_hist(11, 1:iteraciones ))
xlabel('t');
ylabel('error');
title('Error en el eje x');
grid on
hold off
subplot(1,2,2)
hold on
plot(t,qid(2, 1:iteraciones) - z_hist(2, 1:iteraciones))
plot(t,qid(5, 1:iteraciones) - z_hist(5, 1:iteraciones))
plot(t,qid(8, 1:iteraciones) - z_hist(8, 1:iteraciones))
plot(t,qid(11, 1:iteraciones) - z_hist(11, 1:iteraciones))
xlabel('t');
ylabel('error');
title('Error en el eje y');
grid on 
hold off

figure 
subplot(1,2,1)
hold on 
plot(t,vi(1,1:iteraciones))
plot(t,vi(2,1:iteraciones))
plot(t,vi(3,1:iteraciones))
plot(t,vi(4,1:iteraciones))
xlabel('t');
ylabel('v');
title('Ley de control v');
grid on
hold off
subplot(1,2,2)
hold on
plot(t,wi(1,1:iteraciones))
plot(t,wi(2,1:iteraciones))
plot(t,wi(3,1:iteraciones))
plot(t,wi(4,1:iteraciones))
xlabel('t');
ylabel('w');
title('Ley de control w');
grid on 
hold off

figure;
hold on;
for p = 1:num_agentes
    plot(0:h:tiempo, qid(3*p-1, 1:iteraciones + 1) - z_hist(3*p-1, 1:iteraciones + 1), 'DisplayName', ['Error y Agente ', num2str(p)]);
end
xlabel('Tiempo (s)');
ylabel('Error en y');
title('Error en y de los agentes con respecto al tiempo');
legend('show');
grid on;
hold off;




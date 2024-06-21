clc;
clearvars;
close all;

L = [1 -1 0 0 -1;
    -1 2 -1 0 0;
     0 -1 1 0 0;
     0 0 -1 1 0;
     0 0  0 0 0];

% Condiciones iniciales para las coordenadas por separado
x_x = [0; 0; 0; 0]; % Coordenadas x
x_y = [0; 0; 0; 4]; % Coordenadas y

% Inicializar el vector combinado
x = zeros(2 * length(x_x), 1);

% Combinar los vectores para la simulación
for i = 1:length(x_x)
    x(2*i-1) = x_x(i);
    x(2*i) = x_y(i);
end


% Desplazamientos relativos (deltas) entre los agentes
delta_ji = [0 2; -2 0; 0 -2; 2 0]; % [delta_1; delta_2; delta_3; delta_4]
delta = reshape(delta_ji', [], 1); % Convertir las deltas a un vector columna

%Generar el archivo de tiempo
tiempo = 100;
t=linspace(0, tiempo, 1000);

% Función de referencia virtual xi^r
xi_r = [30*sin(pi*t/100); 20*sin(pi*t/50)];

%derivada de la referencia
f = [30*(pi/100)*cos(pi*t/100); 20*(pi/50)*cos(pi*t/50)];

% Parámetros constantes para el controlador
alpha_i = 1;
gamma_i = 1;
k_ij = 1;

% Tiempo total de simulación y período de muestreo
Dt = 0.01;
iteraciones = tiempo / Dt;

% Ganancia proporcional
k_p = 1;

% Inicializar la matriz de estados para almacenar los resultados de la simulación
xi = zeros(size(x, 1), iteraciones + 1);
xi(:, 1) = x;

% Inicializar el vector de desplazamientos y referencia virtual
delta_dot = zeros(size(delta));





% Simulación con el controlador especificado
for k = 1:iteraciones
    % Calcular el término de controlador para cada agente
    u = zeros(size(x),2);
    eta = zeros(size(x, 1), 1);
    
    for i = 1:size(L, 1)
        for j = 1:size(L, 2)
            if L(i, j) ~= 0 % Hay una conexión entre i y j
                g_ij=1;
            end
            if L(i,end) ~= 0 %Hay una conexión entre la referencia y el vehículo
                g_in1=1;
            end
            eta(i)=g_in1*alpha_i+g_ij*k_ij;
            u(i:i)=delta_dot(i)+(1/eta(i))*(g_ij*k_ij*(xi(j,k+1)-delta_dot(j)-gamma_i*((xi(i,k)-xi(j,k))-(delta(i)-delta(j)))))+(1/eta(i))*g_in1*alpha_i*(f(:,k)-gamma_i*(xi(i,k)-delta(i)-xi_r(:,k)));
        end
    end
    
    % Aproximación de Euler para los estados
    xi(:, k+1) = xi(:, k) + Dt * u;
  
end


% Vector de tiempo para graficar
t = linspace(0, tiempo, iteraciones + 1);

% Subplot para consenso en el eje x
figure;
subplot(2, 1, 1);
hold on;
for i = 1:2:size(xi, 1) % Consenso en x
     plot(t, xi(i, :));
end
hold off;
xlabel('Tiempo (s)');
ylabel('\xi_{x}');
title('Consenso en el eje x');
grid on;

% Subplot para consenso en el eje y
subplot(2, 1, 2);
hold on;
for i = 2:2:size(xi, 1) % Consenso en y
    plot(t, xi(i, :));
end
hold off;
xlabel('Tiempo (s)');
ylabel('\xi_{y}');
title('Consenso en el eje y');
grid on;

% Graficar trayectorias en el plano x-y
figure;
hold on;
colors = ['r', 'g', 'b', 'm']; % Colores diferentes para cada agente
for i = 1:length(x_x)
    plot(xi(2*i-1, :), xi(2*i, :), 'Color', colors(i), 'DisplayName', ['Agente ' num2str(i)]);
end
hold off;
xlabel('Posición en x');
ylabel('Posición en y');
title('Trayectorias de los agentes en el plano x-y');
legend show;
grid on;

% Graficar la referencia virtual xi^r
figure;
plot(t, xi_r(1:end-1), 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('\xi^r');
title('Referencia Virtual \xi^r(t)');
grid on;

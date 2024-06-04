clc
clearvars
close all

% Parámetros
num_agents = 4; % Número de agentes
T = 20; % Tiempo de simulación
Dt = 0.01; % Paso de tiempo
iteraciones = T / Dt;

 % Matriz de adyacencia 
A1 = [0 0 0 0;
      1 0 1 0;
      0 1 0 0;
      1 0 0 0;];
% Matriz de adyacencia segundo caso
% A1=[0 1 0 1;
%     1 0 1 0;
%     0 1 0 0;
%     1 0 0 0];

K = ones(num_agents); % Matriz de coeficientes k_{ij}
alpha = ones(num_agents, 1); % Vector de coeficientes alpha_i
gamma = 50*ones(num_agents, 1); % Vector de coeficientes gamma_i

% Inicialización
xi = zeros(num_agents, iteraciones); % Estados de los agentes
xi_dot = zeros(num_agents, iteraciones); % Derivadas de los estados

% Inicialización de los estados de los agentes
xi(:, 1) = [1.5;.5;0;-0.5]; % Estados iniciales estados

% Primera referencia
xi_r1 = cos(0:Dt:(iteraciones-1)*Dt); % Primera referencia variante con el tiempo

% Dinámica de la primera referencia
f1 =  @(t, xi_r) -sin(t); % Derivada de la referencia

% Simulación con la primera referencia
xi1 = xi; % Copia de los estados iniciales
for k = 1:iteraciones-1
    t = (k-1) * Dt;
    
    % Derivada de la referencia en el tiempo actual
    xi_r1_dot = f1(t, xi_r1(k));
    
    for i = 1:num_agents
        if i == 1 % Vehículo con acceso a la referencia
            u_i = xi_r1_dot - sum(A1(i, :) .* K(i, :) .* (xi1(i, k) - xi1(:, k)')) - alpha(i) * (xi1(i, k) - xi_r1(k));
        else % Otros vehículos
            sum_gk = sum(A1(i, :) .* K(i, :));
            if sum_gk == 0
                u_i = 0;
            else
                u_i = (1 / sum_gk) * sum(A1(i, :) .* K(i, :) .* (xi_dot(:, k)' - gamma(i) * (xi1(i, k) - xi1(:, k)')));
            end
        end
        xi_dot(i, k) = u_i;
    end
    
    % Actualización de los estados usando la aproximación de Euler
    xi1(:, k+1) = xi1(:, k) + Dt * xi_dot(:, k);
end

% Segunda referencia
xi_r2 = zeros(1, iteraciones);
xi_r2(1) = 0.5; % Estado inicial de la segunda referencia

% Dinámica de la segunda referencia
f2 = @(t, xi_r) sin(t) * sin(2 * xi_r); % Derivada de la segunda referencia

% Simulación con la segunda referencia
xi2 = xi; % Copia de los estados iniciales
for k = 1:iteraciones-1
    t = (k-1) * Dt;
    
    % Derivada de la referencia en el tiempo actual
    xi_r2_dot = f2(t, xi_r2(k));
    
    % Actualización de la segunda referencia
    xi_r2(k+1) = xi_r2(k) + Dt * xi_r2_dot;
    
    for i = 1:num_agents
        if i == 1 % Vehículo con acceso a la referencia
            u_i = xi_r2_dot - sum(A1(i, :) .* K(i, :) .* (xi2(i, k) - xi2(:, k)')) - alpha(i) * (xi2(i, k) - xi_r2(k));
        else % Otros vehículos
            sum_gk = sum(A1(i, :) .* K(i, :));
            if sum_gk == 0
                u_i = 0;
            else
                u_i = (1 / sum_gk) * sum(A1(i, :) .* K(i, :) .* (xi_dot(:, k)' - gamma(i) * (xi2(i, k) - xi2(:, k)')));
            end
        end
        xi_dot(i, k) = u_i;
    end
    
    % Actualización de los estados usando la aproximación de Euler
    xi2(:, k+1) = xi2(:, k) + Dt * xi_dot(:, k);
end

% Ploteo de resultados
time = 0:Dt:(iteraciones-1)*Dt;

figure;

% Subplot para la primera referencia
subplot(2, 1, 1);
hold on;
for i = 1:num_agents
    plot(time, xi1(i, :));
end
plot(time, xi_r1, '--', 'LineWidth', 2); % Primera referencia
xlabel('Tiempo');
ylabel('Estado');
title('Primera referencia');
hold off;

% Subplot para la segunda referencia
subplot(2, 1, 2);
hold on;
for i = 1:num_agents
    plot(time, xi2(i, :));
end
plot(time, xi_r2, ':', 'LineWidth', 2); % Segunda referencia
xlabel('Tiempo');
ylabel('Estado');
title('Segunda referencia');
hold off;


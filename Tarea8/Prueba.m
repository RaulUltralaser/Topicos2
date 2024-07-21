clc;
clear;
close all;

% Parámetros iniciales
N = 4; % Número de agentes
d = 0.1; % Distancia del punto de control al centro del robot
sigma = 0.6; % Distancia de seguridad
gamma = 1; % Ganancia para el control de seguimiento
lambda = 0.8; % Ganancia para la esquiva de obstáculos
k = 0.1; % Ganancia para el consenso

% Condiciones iniciales
qx = [0, -2, -3, -1]';
qy = [0, 1.8, -1, -0.8]';
qtheta = [0, -0.6, 1.14, 0]';

% Condiciones iniciales en una sola matriz
q = [qx, qy, qtheta];

% Objeto estático en el entorno
alpha_o = [0, -2];

% Vector de desplazamiento para la formación deseada
z = 1.5 * [cos(3*pi/2), sin(3*pi/2);
           cos(2*pi), sin(2*pi);
           cos(pi/2), sin(pi/2);
           cos(pi), sin(pi)];

% Gráfica de la formación inicial
figure;
plot(qx, qy, 'o');
hold on;
plot(alpha_o(1), alpha_o(2), 'x', 'MarkerSize', 10, 'LineWidth', 2);
title('Formación Inicial');
xlabel('x');
ylabel('y');
grid on;
axis equal;

% Tiempo de simulación
tau = 200;
Dt = 0.1; % Paso de tiempo
T = 0:Dt:tau;

% Trayectoria de referencia
m = @(t) 4 * [sin(4*pi*t/tau) * cos(2*pi*t/tau);
              sin(4*pi*t/tau) * sin(2*pi*t/tau)];

% Inicializar almacenamiento de resultados
qx_hist = zeros(N, length(T));
qy_hist = zeros(N, length(T));
qtheta_hist = zeros(N, length(T));
qx_hist(:,1) = qx;
qy_hist(:,1) = qy;
qtheta_hist(:,1) = qtheta;

% Matriz Laplaciana para la formación
L = [2 -1  0 -1;
    -1  2 -1  0;
     0 -1  2 -1;
    -1  0 -1  2];

% Simulación
for t_idx = 2:length(T)
    t = T(t_idx);
    alpha = [qx, qy] + z; % Posición de los agentes con el desplazamiento
    alpha_dot = smooth_transition(alpha, alpha_o, sigma, lambda, L, k, gamma);
    qx = qx + Dt * alpha_dot(:,1);
    qy = qy + Dt * alpha_dot(:,2);
    qx_hist(:,t_idx) = qx;
    qy_hist(:,t_idx) = qy;
    qtheta_hist(:,t_idx) = qtheta; % Asumiendo que theta se mantiene constante en este ejemplo
end

% Gráfica de los resultados
figure;
for i = 1:N
    plot(qx_hist(i,:), qy_hist(i,:));
    hold on;
end
plot(alpha_o(1), alpha_o(2), 'x', 'MarkerSize', 10, 'LineWidth', 2);
title('Trayectoria de los agentes');
xlabel('x');
ylabel('y');
grid on;
axis equal;



% Función de transición suave
function alpha_dot = smooth_transition(alpha, alpha_o, sigma, lambda, L, k, gamma)
    N = size(alpha, 1);
    alpha_dot = zeros(N, 2);
    for i = 1:N
        % Esquiva de obstáculos
        alpha_dot_oi = obstacle_avoidance(alpha(i,:), alpha_o, sigma, lambda);
        
        % Control de formación
        alpha_dot_ri = formation_control(alpha, L, k);
        
        % Transición suave
        h = compute_transition_function(alpha(i,:), alpha_o, sigma);
        alpha_dot(i,:) = (1 - h) * alpha_dot_ri + h * alpha_dot_oi;
    end
end

% Función para calcular la función de transición
function h = compute_transition_function(alpha_i, alpha_o, sigma)
    p_io = alpha_i - alpha_o;
    rho = norm(p_io);
    if rho <= sigma
        h = 1;
    else
        h = 0;
    end
end


% Función de control de formación
function alpha_dot = formation_control(alpha, L, k)
    N = size(alpha, 1);
    alpha_dot = zeros(N, 2);
    for i = 1:N
        evi = zeros(1, 2);
        for j = 1:N
            if L(i, j) == 1
                evi = evi + (alpha(j,:) - alpha(i,:));
            end
        end
        alpha_dot(i,:) = k * evi;
    end
end

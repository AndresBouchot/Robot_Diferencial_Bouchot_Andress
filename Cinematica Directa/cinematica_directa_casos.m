clear all; clc; close all;

% =========================================================
% CINEMATICA DIRECTA 
% Calcula [x, y, theta] a partir de [dot_theta_RD, dot_theta_RI]
% Genera: caso_1.png ... caso_8.png, comparacion_trayectorias.png
%         y cinematica_directa.gif (animacion del caso 7)
% =========================================================

% Parametros del robot
L = 0.18;   % distancia entre ruedas [m]
r = 0.03;   % radio de las ruedas [m]
T  = 44;    % tiempo de simulacion [s]
ti = 0.01;  % paso de integracion [s]

% Casos [dot_theta_RD  dot_theta_RI]
casos = [ 10  10;
         -10 -10;
          12   6;
           6  12;
          10 -10;
          10   0;
          10   8;
          10   2 ];

n = T/ti;
X = cell(8,1); Y = cell(8,1); TH = cell(8,1);

for c = 1:8
    dot_theta_RD = casos(c,1);
    dot_theta_RI = casos(c,2);

    VRD = dot_theta_RD * r;
    VRI = dot_theta_RI * r;
    u = (VRD + VRI)/2;   % velocidad lineal [m/s]
    w = (VRD - VRI)/L;   % velocidad angular [rad/s]

    t = zeros(1,n+1); x = zeros(1,n+1); y = zeros(1,n+1); theta = zeros(1,n+1);
    for k = 1:n
        x(k+1) = x(k) + ti*( u*cos(theta(k)) );
        y(k+1) = y(k) + ti*( u*sin(theta(k)) );
        theta(k+1) = theta(k) + ti*w;
        t(k+1) = t(k) + ti;
    end
    X{c} = x; Y{c} = y; TH{c} = theta;

    fprintf("Caso %d: u = %.3f m/s, w = %.4f rad/s, x_f = %.3f, y_f = %.3f, theta_f = %.3f rad\n", ...
        c, u, w, x(end), y(end), theta(end));

    figure(c)
    plot(x, y, 'LineWidth', 3); hold on
    plot(x(1), y(1), 'o', 'MarkerSize', 11, 'LineWidth', 3, 'MarkerFaceColor', 'g');
    plot(x(end), y(end), 's', 'MarkerSize', 11, 'LineWidth', 3, 'MarkerFaceColor', 'r');
    xlabel("POSICION EN X [m]"); ylabel("POSICION EN Y [m]");
    title(sprintf("CASO %d: RD = %g, RI = %g rad/s", c, casos(c,1), casos(c,2)));
    legend("Trayectoria", "Inicio", "Fin"); grid on; axis equal;
    set(gca, 'FontSize', 14);
    exportgraphics(gcf, sprintf('caso_%d.png', c), 'Resolution', 200);
end

% ---------- Comparacion de los 8 casos ----------
figure(9)
hold on
for c = 1:8
    plot(X{c}, Y{c}, 'LineWidth', 2.5);
end
xlabel("POSICION EN X [m]"); ylabel("POSICION EN Y [m]");
title("COMPARACION DE TRAYECTORIAS - 8 CASOS");
legend("Caso 1","Caso 2","Caso 3","Caso 4","Caso 5","Caso 6","Caso 7","Caso 8", ...
       'Location','bestoutside');
grid on; axis equal; set(gca, 'FontSize', 14);
exportgraphics(gcf, 'comparacion_trayectorias.png', 'Resolution', 200);

% ---------- Animacion del caso 7 (curva amplia) ----------
x = X{7}; y = Y{7}; theta = TH{7}; t = 0:ti:T;
figure(10); hold on; grid on; axis equal; set(gca, 'FontSize', 14);

L_robot = 0.25; W_robot = 0.18;
robot_shape = [ L_robot/2,  W_robot/2;  L_robot/2, -W_robot/2;
               -L_robot/2, -W_robot/2; -L_robot/2,  W_robot/2 ]';
wheel = [ -0.06, 0.02; 0.06, 0.02; 0.06, -0.02; -0.06, -0.02 ]';
pos_wheel_L = [0;  W_robot/2 + 0.02];
pos_wheel_R = [0; -W_robot/2 - 0.02];

archivo_gif = 'cinematica_directa.gif';
primer_frame = true;

for k = 1:40:length(t)
    cla
    plot(x, y, '--', 'LineWidth', 1);
    plot(x(1:k), y(1:k), 'b-', 'LineWidth', 3);
    hold on; grid on; axis equal

    Rm = [cos(theta(k)) -sin(theta(k)); sin(theta(k)) cos(theta(k))];
    p = [x(k); y(k)];
    fill_shape = Rm*robot_shape + p;
    wL = Rm*(wheel + pos_wheel_L) + p;
    wR = Rm*(wheel + pos_wheel_R) + p;
    fill(fill_shape(1,:), fill_shape(2,:), [0.8 0.8 0.8]);
    fill(wL(1,:), wL(2,:), [0.1 0.1 0.1]);
    fill(wR(1,:), wR(2,:), [0.1 0.1 0.1]);
    plot(x(k), y(k), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
    frente = Rm*[L_robot/2 + 0.15; 0] + p;
    quiver(x(k), y(k), frente(1)-x(k), frente(2)-y(k), ...
        'LineWidth', 2, 'MaxHeadSize', 2, 'Color', 'r');

    xlabel("POSICION EN X [m]"); ylabel("POSICION EN Y [m]");
    title("ANIMACION CINEMATICA DIRECTA (CASO 7)");
    xlim([min(x)-0.5 max(x)+0.5]); ylim([min(y)-0.5 max(y)+0.5]);
    drawnow

    frame = getframe(gcf); im = frame2im(frame);
    [A, map] = rgb2ind(im, 256);
    if primer_frame
        imwrite(A, map, archivo_gif, 'gif', 'LoopCount', inf, 'DelayTime', 0.05);
        primer_frame = false;
    else
        imwrite(A, map, archivo_gif, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
end

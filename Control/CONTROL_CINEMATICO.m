clear all; clc; close all;

% CONTROL CINEMATICO DE UN ROBOT DIFERENCIAL
% El robot debe llegar a una posicion deseada [xd, yd]
% Se corren DOS simulaciones con las mismas condiciones:
%   sim = 1 ---> SIN saturacion
%   sim = 2 ---> CON saturacion proporcional
% Todas las graficas comparan las dos simulaciones

% TIEMPO DE SIMULACION
ts = 25;          % tiempo total [s]
ti = 0.01;        % paso de integracion [s]
t = 0:ti:ts;

% PARAMETROS DEL ROBOT
L = 0.18;         % distancia entre ruedas [m]
r = 0.03;         % radio de ruedas [m]
thetaDot_max = 20;   % Velocidad angular maxima [rad/s]

% POSICION DESEADA
xd = 5;           % meta en X [m]
yd = 5;           % meta en Y [m]

% GANANCIAS DEL CONTROLADOR
k_rho = .3;      % ganancia de avance
k_theta = 4;    % ganancia de giro

for sim = 1:2

% CONDICIONES INICIALES
x = zeros(1,length(t));       % posicion X [m]
y = zeros(1,length(t));       % posicion Y [m]
theta = zeros(1,length(t));   % orientacion [rad]

% INICIALIZACION DE VARIABLES
rho = zeros(1,length(t));
e_theta = zeros(1,length(t));

u = zeros(1,length(t));
w = zeros(1,length(t));
u_real =  zeros(1,length(t));
w_real =  zeros(1,length(t));

thetaDot_D = zeros(1,length(t));
thetaDot_I = zeros(1,length(t));
thetaDot_D_sat = zeros(1,length(t));
thetaDot_I_sat = zeros(1,length(t));
valor_maximo = zeros(1,length(t));

t_llegada = NaN;   % instante en que alcanza la meta [s]
distancia = 0;     % distancia total recorrida [m]

% SIMULACION
for k = 1:length(t)-1

    % ERRORES DE POSICION
    ex = xd - x(k);
    ey = yd - y(k);

    % Distancia hacia la meta
    rho(k) = sqrt(ex^2 + ey^2);

    % Angulo deseado hacia la meta
    theta_d = atan2(ey,ex);

    % Error angular normalizado
    e_theta(k) = atan2(sin(theta_d - theta(k)), cos(theta_d - theta(k)));

    % CONTROL CINEMATICO
    u(k) = k_rho*rho(k);
    w(k) = k_theta*e_theta(k);

%   Detener si llega cerca de la meta
    if rho(k) < 0.03
        u(k) = 0;
        w(k) = 0;
        if isnan(t_llegada)
            t_llegada = t(k);
        end
    end

% -------------------- INICIO saturacion ----------------------------------
% Cinematica inversa: velocidades de las ruedas
thetaDot_D(k) = (u(k) + (L/2)*w(k))/r;
thetaDot_I(k) = (u(k) - (L/2)*w(k))/r;

% Saturacion proporcional
valor_maximo(k) = max(abs([thetaDot_D(k), thetaDot_I(k)]));

if sim == 2 && valor_maximo(k) > thetaDot_max
    alpha = thetaDot_max/valor_maximo(k);

    thetaDot_D_sat(k) = alpha*thetaDot_D(k);
    thetaDot_I_sat(k) = alpha*thetaDot_I(k);
else
    thetaDot_D_sat(k) = thetaDot_D(k);
    thetaDot_I_sat(k) = thetaDot_I(k);
end

% Velocidades reales del robot despues de saturar
u_real(k) = (r/2)*(thetaDot_D_sat(k) + thetaDot_I_sat(k));
w_real(k) = (r/L)*(thetaDot_D_sat(k) - thetaDot_I_sat(k));

distancia = distancia + abs(u_real(k))*ti;

x(k+1) = x(k) + u_real(k)*cos(theta(k))*ti;
y(k+1) = y(k) + u_real(k)*sin(theta(k))*ti;
theta(k+1) = theta(k) + w_real(k)*ti;
% -------------------- FIN saturacion ----------------------------------

end

rho(end) = rho(end-1);
e_theta(end) = e_theta(end-1);
u(end) = u(end-1);
w(end) = w(end-1);
u_real(end) = u_real(end-1);
w_real(end) = w_real(end-1);
thetaDot_D(end) = thetaDot_D(end-1);
thetaDot_I(end) = thetaDot_I(end-1);
thetaDot_D_sat(end) = thetaDot_D_sat(end-1);
thetaDot_I_sat(end) = thetaDot_I_sat(end-1);

% SE GUARDAN LOS RESULTADOS DE ESTA SIMULACION
RES(sim).x = x;
RES(sim).y = y;
RES(sim).theta = theta;
RES(sim).rho = rho;
RES(sim).e_theta = e_theta;
RES(sim).u_real = u_real;
RES(sim).w_real = w_real;
RES(sim).thetaDot_D = thetaDot_D;
RES(sim).thetaDot_I = thetaDot_I;
RES(sim).thetaDot_D_sat = thetaDot_D_sat;
RES(sim).thetaDot_I_sat = thetaDot_I_sat;
RES(sim).error_final = sqrt((xd-x(end))^2 + (yd-y(end))^2);
RES(sim).t_llegada = t_llegada;
RES(sim).distancia = distancia;

end

% TABLA DE COMPARACION DE DESEMPEÑO (seccion 8.2 del reporte)
fprintf('\n         INDICADOR                SIN SATURACION   CON SATURACION\n');
fprintf('Error final (m)                        %6.4f          %6.4f\n', ...
    RES(1).error_final, RES(2).error_final);
fprintf('Tiempo de llegada (s)                  %6.2f          %6.2f\n', ...
    RES(1).t_llegada, RES(2).t_llegada);
fprintf('thetaDot_D maxima aplicada (rad/s)     %6.2f          %6.2f\n', ...
    max(abs(RES(1).thetaDot_D_sat)), max(abs(RES(2).thetaDot_D_sat)));
fprintf('thetaDot_I maxima aplicada (rad/s)     %6.2f          %6.2f\n', ...
    max(abs(RES(1).thetaDot_I_sat)), max(abs(RES(2).thetaDot_I_sat)));
fprintf('Distancia recorrida (m)                %6.3f          %6.3f\n\n', ...
    RES(1).distancia, RES(2).distancia);

%%
% GRAFICA DE TRAYECTORIA
figure(1)
plot(RES(1).x,RES(1).y,'LineWidth',5)
hold on
plot(RES(2).x,RES(2).y,'LineWidth',3)
plot(0,0,'go','MarkerSize',12,'LineWidth',3,'MarkerFaceColor','g')
plot(xd,yd,'rp','MarkerSize',18,'LineWidth',3,'MarkerFaceColor','r')

xlabel("Posicion X [m]")
ylabel("Posicion Y [m]")
title("Control cinematico de robot diferencial")
legend("Sin saturacion","Con saturacion","Inicio","Meta")
grid on
axis equal
set(gca,'FontSize',16)
exportgraphics(gcf, 'comparacion_saturacion.png', 'Resolution', 200);

% GRAFICAS DE ESTADOS
figure(2)

subplot(3,1,1)
plot(t,RES(1).x,'LineWidth',3); hold on
plot(t,RES(2).x,'LineWidth',3)
grid on
ylabel("x [m]")
title("Posicion en X")
legend("Sin saturacion","Con saturacion")

subplot(3,1,2)
plot(t,RES(1).y,'LineWidth',3); hold on
plot(t,RES(2).y,'LineWidth',3)
grid on
ylabel("y [m]")
title("Posicion en Y")

subplot(3,1,3)
plot(t,RES(1).theta,'LineWidth',3); hold on
plot(t,RES(2).theta,'LineWidth',3)
grid on
ylabel("theta [rad]")
xlabel("Tiempo [s]")
title("Orientacion del robot")
exportgraphics(gcf, 'estados.png', 'Resolution', 200);

% GRAFICAS DE ERRORES
figure(3)

subplot(2,1,1)
plot(t,RES(1).rho,'LineWidth',3); hold on
plot(t,RES(2).rho,'LineWidth',3)
grid on
ylabel("rho [m]")
title("Distancia a la meta")
legend("Sin saturacion","Con saturacion")

subplot(2,1,2)
plot(t,RES(1).e_theta,'LineWidth',3); hold on
plot(t,RES(2).e_theta,'LineWidth',3)
grid on
ylabel("e theta [rad]")
xlabel("Tiempo [s]")
title("Error angular")
exportgraphics(gcf, 'errores.png', 'Resolution', 200);

% GRAFICAS DE CONTROL
figure(4)

subplot(2,1,1)
plot(t,RES(1).u_real,'LineWidth',3); hold on
plot(t,RES(2).u_real,'LineWidth',3)
grid on
ylabel("u [m/s]")
title("Velocidad lineal aplicada al robot")
legend("Sin saturacion","Con saturacion")

subplot(2,1,2)
plot(t,RES(1).w_real,'LineWidth',3); hold on
plot(t,RES(2).w_real,'LineWidth',3)
grid on
ylabel("w [rad/s]")
xlabel("Tiempo [s]")
title("Velocidad angular aplicada al robot")
exportgraphics(gcf, 'velocidades_robot.png', 'Resolution', 200);

% GRAFICAS DE VELOCIDADES DE RUEDAS
% Se compara lo que PIDE el controlador contra lo que se APLICA tras saturar
figure(5)
subplot(2,1,1)
plot(t,RES(2).thetaDot_D, t,RES(2).thetaDot_D_sat, 'LineWidth',3)
hold on
yline(thetaDot_max,'r--','LineWidth',2);
yline(-thetaDot_max,'r--','LineWidth',2);
grid on
ylabel("$\dot{\theta}_D$ [rad/s]", Interpreter="latex")
title("Velocidad angular rueda derecha")
legend("Demandada","Saturada","Limite")
set(gca,'FontSize',16)

subplot(2,1,2)
plot(t,RES(2).thetaDot_I, t,RES(2).thetaDot_I_sat,'LineWidth',3)
hold on
yline(thetaDot_max,'r--','LineWidth',2);
yline(-thetaDot_max,'r--','LineWidth',2);
grid on
ylabel("$\dot{\theta}_I$ [rad/s]", Interpreter="latex")
xlabel("Tiempo [s]")
title("Velocidad angular rueda izquierda")
set(gca,'FontSize',16)
exportgraphics(gcf, 'velocidades_ruedas.png', 'Resolution', 200);

% ANIMACION DEL ROBOT DIFERENCIAL (LAS DOS SIMULACIONES A LA VEZ)
figure(6)
hold on
grid on
axis equal
xlabel("Posicion X [m]")
ylabel("Posicion Y [m]")
title("CONTROL CINEMATICO: SIN Y CON SATURACION")
set(gca, 'FontSize', 16);
% Dimensiones visuales del robot
L_robot = 0.4; % largo del robot
W_robot = 0.28; % ancho del robot
% Forma del chasis en el marco local del robot
robot_shape = [
    L_robot/2, W_robot/2;
    L_robot/2, -W_robot/2;
    -L_robot/2, -W_robot/2;
    -L_robot/2, W_robot/2
    ]';
% Ruedas en el marco local
wheel_L = [
    -0.1, 0.03;
    0.1, 0.03;
    0.1, -0.03;
    -0.1, -0.03
    ]';
wheel_R = wheel_L;
% Posición lateral de las ruedas
pos_wheel_L = [0; W_robot/2 + 0.03];
pos_wheel_R = [0; -W_robot/2 - 0.03];

nombre_gif = 'control_cinematico.gif';
primer_cuadro = 1;

for k = 1:25:length(t) %define velocidad
    cla
    % Meta
    plot(xd,yd,'rp','MarkerSize',18,'LineWidth',3,'MarkerFaceColor','r');
    hold on
    % Trayectorias recorridas hasta el instante actual
    plot(RES(1).x(1:k), RES(1).y(1:k), 'LineWidth', 4);
    plot(RES(2).x(1:k), RES(2).y(1:k), 'LineWidth', 2);
    grid on
    axis equal

    % Se dibujan los dos robots
    for sim = 1:2
        % Matriz de rotación
        R = [
            cos(RES(sim).theta(k)) -sin(RES(sim).theta(k));
            sin(RES(sim).theta(k)) cos(RES(sim).theta(k))
            ];
        % Posición actual del robot
        p = [RES(sim).x(k); RES(sim).y(k)];
        % Rotar y trasladar chasis y ruedas
        robot_global = R*robot_shape + p;
        wheel_L_global = R*(wheel_L + pos_wheel_L) + p;
        wheel_R_global = R*(wheel_R + pos_wheel_R) + p;
        % Dibujar chasis
        if sim == 1
            fill(robot_global(1,:), robot_global(2,:), [0.85 0.85 0.85]);
        else
            fill(robot_global(1,:), robot_global(2,:), [0.95 0.75 0.55]);
        end
        % Dibujar ruedas
        fill(wheel_L_global(1,:), wheel_L_global(2,:), [0.1 0.1 0.1]);
        fill(wheel_R_global(1,:), wheel_R_global(2,:), [0.1 0.1 0.1]);
        % Dibujar dirección frontal del robot
        frente = R*[L_robot/2 + 0.3; 0] + p;
        quiver(RES(sim).x(k), RES(sim).y(k), ...
            frente(1)-RES(sim).x(k), frente(2)-RES(sim).y(k), ...
            'LineWidth',2,'MaxHeadSize',2);
    end

    xlabel("Posicion X [m]")
    ylabel("Posicion Y [m]")
    title("CONTROL CINEMATICO: SIN Y CON SATURACION")
    legend("Meta","Sin saturacion","Con saturacion")
    xlim([-1 6])
    ylim([-1 6])
    set(gca, 'FontSize', 16);
    pause(0.01)

    % GUARDAR EL CUADRO EN EL GIF (lo pide la practica)
    cuadro = getframe(gcf);
    imagen = frame2im(cuadro);
    [A,mapa] = rgb2ind(imagen,256);
    if primer_cuadro == 1
        imwrite(A,mapa,nombre_gif,'gif','LoopCount',Inf,'DelayTime',0.05);
        primer_cuadro = 0;
    else
        imwrite(A,mapa,nombre_gif,'gif','WriteMode','append','DelayTime',0.05);
    end
end

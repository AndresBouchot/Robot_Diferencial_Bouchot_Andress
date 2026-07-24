clear all; close all; clc;
%codigo para analizar CINEMATICA INVERSA DE UN ROBOT DIFERENCIAL 
% Condiciones iniciales
t(1) = 0;
x(1) = 0;        % posicion inicial en x [m]
y(1) = 0;        % posicion inicial en y [m]
theta(1) = pi/2;    % orientacion inicial [rad]

% Parametros del carrito diferencial
L = 0.18;        % distancia entre ruedas [m]
r = 0.03;        % radio de las ruedas [m]

% VELOCIDADES DESDE MARCO INERCIAL ... CIRCULO
R = 2;              % radio circulo [m]
omega_c = 0.25;     % velocidad angular deseada [rad/s]

% Tiempo de simulacion
ts = 2*pi/omega_c;  % tiempo de una vuelta completa [s]  (25.13 s)
ti = 0.01;       % paso de integracion metodo Euler [s]
t = 0:ti:ts;

%inicializar variable de u y w
u = zeros(1, length(t)); %vector para datos en u
w = zeros(1, length(t)); %vector para datos en w
VD = zeros(1, length(t)); %vector para vel lineal rueda derecha
VI = zeros(1, length(t)); %vector para vel lineal rueda izquierda
dot_theta_RD = zeros(1, length(t)); %vector para vel angular rueda derecha
dot_theta_RI = zeros(1, length(t)); %vector para vel angular rueda izquierda
RPM_D = zeros(1, length(t));
RPM_I = zeros(1, length(t));
error_tray = zeros(1, length(t));

% TRAYECTORIA DESEADA
xd = zeros(1, length(t));
yd = zeros(1, length(t));
theta_d = zeros(1, length(t));
theta_d(1) = pi/2;

% CONOCEMOS VELOCIDADES DESDE MARCO INERCIAL 
% x_punto =  1.5 + 0.5*sin(0.5*t );
% y_punto = cos(0.5*t) ; 
% theta_punto = 0.4*sin(0.3*t);

x_punto = -R*omega_c*sin(omega_c*t);
y_punto =  R*omega_c*cos(omega_c*t);
theta_punto = omega_c*ones(size(t));

% TRAYECTORIA DESEADA (se integran las velocidades del marco inercial)
for k = 1: length(t)-1
    xd(k+1) = xd(k) + ti* x_punto(k);
    yd(k+1) = yd(k) + ti* y_punto(k);
    theta_d(k+1) = theta_d(k) + ti* theta_punto(k) ;
end

%CINEMATICA INVERSA
for k = 1: length(t)-1
    % OJO: aqui va theta_d(k), la ORIENTACION, no theta_punto(k)
    u(k) = x_punto(k) * cos( theta_d(k) ) + y_punto(k)* sin(theta_d(k));
    w(k) = theta_punto(k);

    VD(k) = u(k) + (L/2)* w(k); %VEL LINEAL rueda derecha (m/s)
    VI(k) = u(k) - (L/2)* w(k); %VEL LINEAL rueda izquierda (m/s)
    dot_theta_RD(k) = VD(k)/ r; %VEL ANGULAR RUEDA DERECHA (RAD/S)
    dot_theta_RI(k) = VI(k)/ r; %VEL ANGULAR RUEDA IZQUIERDA (RAD/S)
    RPM_D(k) = 60 * dot_theta_RD(k) / (2*pi) ;
    RPM_I(k) = 60 * dot_theta_RI(k) / (2*pi) ;

    % SE COMPRUEBA CON LA CINEMATICA DIRECTA: se le meten al robot las
    % velocidades calculadas y se ve si reproduce la trayectoria deseada
    x(k+1) = x(k) + ti*( u(k)*cos(theta(k)) );
    y(k+1) = y(k) + ti*( u(k)*sin(theta(k)) );
    theta(k+1) = theta(k) + ti* w(k) ;

    error_tray(k) = sqrt( (x(k)-xd(k))^2 + (y(k)-yd(k))^2 );
end

u(end) = u(end-1);
w(end) = w(end-1);
VD(end) = VD(end-1);
VI(end) = VI(end-1);
dot_theta_RD(end) = dot_theta_RD(end-1);
dot_theta_RI(end) = dot_theta_RI(end-1);
RPM_D(end) = RPM_D(end-1);
RPM_I(end) = RPM_I(end-1);
error_tray(end) = error_tray(end-1);

% RESULTADOS PARA LA TABLA DE LA PRACTICA
disp("u minima y maxima (m/s) =")
disp([min(u) max(u)])
disp("w minima y maxima (rad/s) =")
disp([min(w) max(w)])
disp("dot_theta_RD minima y maxima (rad/s) =")
disp([min(dot_theta_RD) max(dot_theta_RD)])
disp("dot_theta_RI minima y maxima (rad/s) =")
disp([min(dot_theta_RI) max(dot_theta_RI)])
disp("Error de trayectoria minimo y maximo (m) =")
disp([min(error_tray) max(error_tray)])
disp("RPM de cada rueda (D , I) =")
disp([max(RPM_D) max(RPM_I)])

% ------------ GRAFICAS ----------------
figure(1)
subplot(3,1,1)
plot(t,x_punto, 'LineWidth',5)
grid on; title("Velocidad deseada en x");
xlabel("tiempo (s)"); ylabel(" $\dot{x}$ (m/s)", "Interpreter","latex");set(gca,'FontSize',20)

subplot(3,1,2)
plot(t,y_punto, 'LineWidth',5)
grid on; title("Velocidad deseada en y");
xlabel("tiempo (s)"); ylabel(" $\dot{y}$ (m/s)", "Interpreter","latex");set(gca,'FontSize',20)

subplot(3,1,3)
plot(t,theta_punto, 'LineWidth',5)
grid on; title("Velocidad deseada en THETA");
xlabel("tiempo (s)"); ylabel(" $\dot{\theta}$ (rad/s)", "Interpreter","latex");
set(gca,'FontSize',20)

figure(2)
subplot( 2,1,1)
plot(t,u , 'LineWidth', 5)
grid on; title("Velocidad lineal robot")
xlabel("tiempo (s)"); ylabel(" u (m/s)");
set(gca,'FontSize',16)

subplot( 2,1,2)
plot(t,w , 'LineWidth', 5)
grid on; title("Velocidad angular robot")
xlabel("tiempo (s)"); ylabel(" w (rad/s)");
set(gca,'FontSize',16)
exportgraphics(gcf, 'velocidades_robot.png', 'Resolution', 200);

figure(3)
plot(xd,yd,'k--','LineWidth',5)
hold on
plot(x,y,'LineWidth',4)
plot(x(1),y(1),'go','MarkerSize',12,'LineWidth',3,'MarkerFaceColor','g')
plot(x(end),y(end),'rs','MarkerSize',12,'LineWidth',3,'MarkerFaceColor','r')

xlabel("Posicion X [m]")
ylabel("Posicion Y [m]")
title("Trayectoria deseada y trayectoria obtenida")
legend("Deseada","Obtenida","Inicio","Final")
grid on
axis equal
set(gca,'FontSize',16)
exportgraphics(gcf, 'comparacion_trayectorias.png', 'Resolution', 200);

figure(4)
subplot( 2,1,1)
plot(t,VD , 'LineWidth', 5); hold on;
plot(t,VI , 'LineWidth', 5); hold on;
grid on; title("Velocidad lineal de cada rueda")
legend("VD", "VI")
xlabel("tiempo (s)"); ylabel(" VD Y VI (m/s)");
set(gca,'FontSize',16)

subplot( 2,1,2)
plot(t, dot_theta_RD , 'LineWidth', 5); hold on;
plot(t,dot_theta_RI , 'LineWidth', 5); hold on;
grid on; title("Velocidad angular de cada rueda")
legend("$ \dot{\theta}_D $", "$ \dot{\theta}_I$","Interpreter","latex")
xlabel("tiempo (s)"); ylabel("$ \dot{\theta} $ (rad/s)", "Interpreter","latex");
set(gca,'FontSize',20)
exportgraphics(gcf, 'velocidades_ruedas.png', 'Resolution', 200);

figure(5)
plot(t, RPM_D, 'LineWidth',5); hold on
plot(t, RPM_I, 'LineWidth',5);
grid on
legend("RPM rueda derecha","RPM rueda izquierda")
xlabel("tiempo (s)"); ylabel("RPM")
set(gca,'FontSize',16)
title("RPM PARA CADA RUEDA")
exportgraphics(gcf, 'rpm_ruedas.png', 'Resolution', 200);

figure(6)
plot(t, error_tray, 'LineWidth',5)
grid on
xlabel("tiempo (s)"); ylabel("error (m)")
title("Error entre la trayectoria deseada y la obtenida")
set(gca,'FontSize',16)
exportgraphics(gcf, 'error_trayectoria.png', 'Resolution', 200);

% ANIMACION DEL ROBOT DIFERENCIAL
figure(7)
hold on
grid on
axis equal
xlabel("Posicion X [m]")
ylabel("Posicion Y [m]")
title("CINEMATICA INVERSA - TRAYECTORIA CIRCULAR")
set(gca, 'FontSize', 16);
% Dimensiones visuales del robot
L_robot = 0.35; % largo del robot
W_robot = 0.25; % ancho del robot
% Forma del chasis en el marco local del robot
robot_shape = [
    L_robot/2, W_robot/2;
    L_robot/2, -W_robot/2;
    -L_robot/2, -W_robot/2;
    -L_robot/2, W_robot/2
    ]';
% Ruedas en el marco local
wheel_L = [
    -0.09, 0.03;
    0.09, 0.03;
    0.09, -0.03;
    -0.09, -0.03
    ]';
wheel_R = wheel_L;
% Posición lateral de las ruedas
pos_wheel_L = [0; W_robot/2 + 0.03];
pos_wheel_R = [0; -W_robot/2 - 0.03];

nombre_gif = 'cinematica_inversa.gif';
primer_cuadro = 1;

for k = 1:25:length(t) %define velocidad
    cla
    % Trayectoria deseada de referencia
    plot(xd,yd,'k--','LineWidth',1);
    % Trayectoria recorrida hasta el instante actual
    plot(x(1:k),y(1:k),'LineWidth',3);
    hold on
    grid on
    axis equal
    % Matriz de rotación
    Rm = [
        cos(theta(k)) -sin(theta(k));
        sin(theta(k)) cos(theta(k))
        ];
    % Posición actual del robot
    p = [x(k); y(k)];
    % Rotar y trasladar chasis
    robot_global = Rm*robot_shape + p;
    % Rotar y trasladar ruedas
    wheel_L_global = Rm*(wheel_L + pos_wheel_L) + p;
    wheel_R_global = Rm*(wheel_R + pos_wheel_R) + p;
    % Dibujar chasis
    fill(robot_global(1,:), robot_global(2,:), [0.8 0.8 0.8]);
    % Dibujar ruedas
    fill(wheel_L_global(1,:), wheel_L_global(2,:), [0.1 0.1 0.1]);
    fill(wheel_R_global(1,:), wheel_R_global(2,:), [0.1 0.1 0.1]);
    % Dibujar punto central
    plot(x(k),y(k),'ko','MarkerSize',8,'MarkerFaceColor','k');
    % Dibujar dirección frontal del robot
    frente = Rm*[L_robot/2 + 0.3; 0] + p;
    quiver(x(k),y(k),frente(1)-x(k),frente(2)-y(k), ...
        'LineWidth',2,'MaxHeadSize',2);
    xlabel("Posicion X [m]")
    ylabel("Posicion Y [m]")
    title("CINEMATICA INVERSA - TRAYECTORIA CIRCULAR")
    xlim([min(xd)-0.6 max(xd)+0.6])
    ylim([min(yd)-0.6 max(yd)+0.6])
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

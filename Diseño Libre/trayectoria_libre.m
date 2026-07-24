clear all;
clc;
close all;

% PROGRAMA QUE CALCULA [x,y,theta] a partir del [u, w]   
% para un ROBOT DIFERENCIAL
% TRAYECTORIA LIBRE: FIGURA DE INFINITO (OCHO)
% Se forma con dos circunferencias tangentes: la primera girando a la
% izquierda y la segunda girando a la derecha, una vuelta completa cada una

%Condiciones iniciales
t(1)= 0;
x(1) = 0; % vector posicion x
y(1) = 0; % vector posicion y
theta(1) = 0; % vector orientacion theta

ts = 35; %tiempo de simulacion en segundos
ti = 0.01; %tiempo de integracion metdodo euler
% PARAMETROS DEL CARRITO DIFERENCIAL 
L = 0.18; % distancia entre ruedas RD y RI
r = 0.03; % radio RUEDAS RD RI[m]

% CALCULO DE LAS VELOCIDADES DE CADA LAZO
% Para dar una vuelta completa (2*pi rad) en 10 segundos se necesita:
w_lazo = 2*pi/10;      % 0.6283 rad/s
u_lazo = 0.3;          % velocidad de avance deseada [m/s]
% De  u = (VRD+VRI)/2  y  w = (VRD-VRI)/L  con VRD = dot_theta_RD*r
% se despejan las velocidades de la rueda rapida y de la rueda lenta:
rueda_rapida = u_lazo/r + (w_lazo*L)/(2*r);   % 11.885 rad/s
rueda_lenta  = u_lazo/r - (w_lazo*L)/(2*r);   % 8.115 rad/s
R_lazo = u_lazo/w_lazo;                       % radio de cada lazo = 0.4775 m

% EL INFINITO SE RECORRE DOS VECES: LA SEGUNDA MAS RAPIDO
% Si se multiplican LAS DOS ruedas por el mismo factor, u y w crecen igual,
% el radio R = u/w no cambia y el robot repite el mismo camino mas rapido
factor = 4/3;
rueda_rapida2 = rueda_rapida*factor;   % 15.847 rad/s
rueda_lenta2  = rueda_lenta*factor;    % 10.820 rad/s
T_lazo2 = 2*pi/(w_lazo*factor);        % 7.5 s por lazo

% SEÑAL DE ENTRADA DE CARRITO DIFERENCIAL 
for k= 1: ts/ti
%cambio de velocidades por intervalos de tiempo
if t(k) < 10
    dot_theta_RD = rueda_rapida;    % lazo 1: giro a la izquierda
    dot_theta_RI = rueda_lenta;
elseif t(k) < 20
    dot_theta_RD = rueda_lenta;     % lazo 2: giro a la derecha
    dot_theta_RI = rueda_rapida;
elseif t(k) < 20 + T_lazo2
    dot_theta_RD = rueda_rapida2;   % lazo 3: izquierda, mas rapido
    dot_theta_RI = rueda_lenta2;
else 
    dot_theta_RD = rueda_lenta2;    % lazo 4: derecha, mas rapido
    dot_theta_RI = rueda_rapida2;
end

VRD = dot_theta_RD * r;
VRI = dot_theta_RI * r;
u = (VRD + VRI) / 2; % m/s
w = (VRD - VRI) / L; % rad/s

    x(k+1) = x(k) + ti*( u*cos(theta(k) ));
    y(k+1) = y(k) + ti*( u*sin(theta(k) ));
    theta(k+1) = theta(k) + ti*(w) ;
    t(k+1) = t(k) + ti;
end

% BOCETO DE LA TRAYECTORIA PROPUESTA
% Son dos circunferencias de radio R_lazo tangentes en el origen:
% la de arriba se recorre a la izquierda y la de abajo a la derecha
ang1 = linspace(-pi/2, 3*pi/2, 300);      % lazo superior
bx1 = R_lazo*cos(ang1);
by1 = R_lazo + R_lazo*sin(ang1);
ang2 = linspace(pi/2, -3*pi/2, 300);      % lazo inferior
bx2 = R_lazo*cos(ang2);
by2 = -R_lazo + R_lazo*sin(ang2);
bx = [bx1 bx2];
by = [by1 by2];

% TABLA DE VELOCIDADES UTILIZADAS
fprintf('\n INTERVALO   t inicial   t final   dot_theta_RD   dot_theta_RI   MOVIMIENTO\n');
fprintf('     1          0.0       10.0        %7.3f        %7.3f      Lazo izquierdo\n', rueda_rapida, rueda_lenta);
fprintf('     2         10.0       20.0        %7.3f        %7.3f      Lazo derecho\n', rueda_lenta, rueda_rapida);
fprintf('     3         20.0       %4.1f        %7.3f        %7.3f      Lazo izquierdo rapido\n', 20+T_lazo2, rueda_rapida2, rueda_lenta2);
fprintf('     4         %4.1f       %4.1f        %7.3f        %7.3f      Lazo derecho rapido\n\n', 20+T_lazo2, ts, rueda_lenta2, rueda_rapida2);

disp("RADIO DE CADA LAZO (m) =")
disp(R_lazo)
disp("POSICION FINAL [x y] (m) =")
disp([x(end) y(end)])
disp("ERROR DE CIERRE (m) =")
disp(norm([x(end) y(end)]))
disp("ORIENTACION FINAL (grados) =")
disp(theta(end)*180/pi)

% BOCETO DE LA TRAYECTORIA PROPUESTA
figure(1)
plot(bx,by,'k--','LineWidth',4);
hold on
plot(0,0,'o','MarkerSize',12,'LineWidth',3,'MarkerFaceColor','g');
xlabel("POSICION EN X [m]")
ylabel("POSICION EN Y [m]")
title("BOCETO DE LA TRAYECTORIA PROPUESTA")
legend("Trayectoria propuesta","Punto de partida")
grid on
axis equal
set(gca, 'FontSize', 18);
exportgraphics(gcf, 'boceto_propuesto.png', 'Resolution', 200);

% TRAYECTORIA OBTENIDA EN MATLAB
figure(2)
plot(x,y,'LineWidth',5);
hold on
plot(x(1),y(1),'o','MarkerSize',12,'LineWidth',3,'MarkerFaceColor','g');
plot(x(end),y(end),'s','MarkerSize',12,'LineWidth',3,'MarkerFaceColor','r');
xlabel("POSICION EN X [m]")
ylabel("POSICION EN Y [m]")
title("TRAYECTORIA OBTENIDA EN MATLAB")
legend("Trayectoria","Inicio","Fin")
grid on
axis equal
set(gca, 'FontSize', 18);
exportgraphics(gcf, 'trayectoria_obtenida.png', 'Resolution', 200);

% COMPARACION ENTRE LA PROPUESTA Y LA OBTENIDA
figure(3)
plot(bx,by,'k--','LineWidth',4);
hold on
plot(x,y,'LineWidth',3);
plot(x(1),y(1),'o','MarkerSize',12,'LineWidth',3,'MarkerFaceColor','g');
plot(x(end),y(end),'s','MarkerSize',12,'LineWidth',3,'MarkerFaceColor','r');
xlabel("POSICION EN X [m]")
ylabel("POSICION EN Y [m]")
title("TRAYECTORIA LIBRE: INFINITO")
legend("Propuesta","Obtenida","Inicio","Fin",'Location','eastoutside')
grid on
axis equal
set(gca, 'FontSize', 18);
exportgraphics(gcf, 'trayectoria_libre.png', 'Resolution', 200);

figure(4)
subplot(3,1,1)
plot(t,x,'LineWidth',5); grid on;
ylabel("posicion x (m)"); xlabel("tiempo (s)");
subplot(3,1,2)
plot(t,y,'LineWidth',5); grid on;
ylabel("posicion y (m)"); xlabel("tiempo (s)");
subplot(3,1,3)
plot(t,theta,'LineWidth',5); grid on;
ylabel("orientacion (radianes)"); xlabel("tiempo (s)");

% ANIMACION DEL ROBOT DIFERENCIAL
figure(5)
hold on
grid on
axis equal
xlabel("POSICION EN X [m]")
ylabel("POSICION EN Y [m]")
title("MOVIMIENTO DEL ROBOT DIFERENCIAL")
set(gca, 'FontSize', 18);
% Dimensiones visuales del robot
L_robot = 0.25; % largo del robot
W_robot = 0.18; % ancho del robot
% Forma del chasis en el marco local del robot
robot_shape = [
    L_robot/2, W_robot/2;
    L_robot/2, -W_robot/2;
    -L_robot/2, -W_robot/2;
    -L_robot/2, W_robot/2
    ]';
% Ruedas en el marco local
wheel_L = [
    -0.06, 0.02;
    0.06, 0.02;
    0.06, -0.02;
    -0.06, -0.02
    ]';
wheel_R = wheel_L;
% Posición lateral de las ruedas
pos_wheel_L = [0; W_robot/2 + 0.02];
pos_wheel_R = [0; -W_robot/2 - 0.02];

nombre_gif = 'trayectoria_libre.gif';
primer_cuadro = 1;

for k = 1:15:length(t) %define velocidad
    cla
    % Boceto de referencia
    plot(bx,by,'k--','LineWidth',1);
    % Trayectoria recorrida hasta el instante actual
    plot(x(1:k),y(1:k),'LineWidth',3);
    hold on
    grid on
    axis equal
    % Matriz de rotación
    R = [
        cos(theta(k)) -sin(theta(k));
        sin(theta(k)) cos(theta(k))
        ];
    % Posición actual del robot
    p = [x(k); y(k)];
    % Rotar y trasladar chasis
    robot_global = R*robot_shape + p;
    % Rotar y trasladar ruedas
    wheel_L_global = R*(wheel_L + pos_wheel_L) + p;
    wheel_R_global = R*(wheel_R + pos_wheel_R) + p;
    % Dibujar chasis
    fill(robot_global(1,:), robot_global(2,:), [0.8 0.8 0.8]);
    % Dibujar ruedas
    fill(wheel_L_global(1,:), wheel_L_global(2,:), [0.1 0.1 0.1]);
    fill(wheel_R_global(1,:), wheel_R_global(2,:), [0.1 0.1 0.1]);
    % Dibujar punto central
    plot(x(k),y(k),'ko','MarkerSize',8,'MarkerFaceColor','k');
    % Dibujar dirección frontal del robot
    frente = R*[L_robot/2 + 0.15; 0] + p;
    quiver(x(k),y(k),frente(1)-x(k),frente(2)-y(k), ...
        'LineWidth',2,'MaxHeadSize',2);
    xlabel("POSICION EN X [m]")
    ylabel("POSICION EN Y [m]")
    title("MOVIMIENTO DEL ROBOT DIFERENCIAL")
    xlim([min(x)-0.3 max(x)+0.3])
    ylim([min(y)-0.3 max(y)+0.3])
    set(gca, 'FontSize', 18);
    pause(0.01)

    % GUARDAR EL CUADRO EN EL GIF
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
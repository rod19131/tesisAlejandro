%% Conectarse
clear;

no_robot = 4;  
robot = robotat_3pi_connect(no_robot);
robotat = robotat_connect('192.168.50.200');
%% Desconectarse
% robotat_3pi_disconnect(robot);
% robotat_disconnect(robotat);
%% Parado de emergencia
robotat_3pi_force_stop(robot);

%% Condiciones del robot
TIME_STEP = 64;
MAX_WHEEL_VELOCITY = 500; % velocidad máxima ruedas (en rad/s)
WHEEL_RADIUS = 17; % radio de las ruedas (en mm)
DISTANCE_FROM_CENTER = 90; % distancia a ruedas (en mm)
% Velocidad lineal máxima (en mm/s)
MAX_SPEED = WHEEL_RADIUS * MAX_WHEEL_VELOCITY;


%% Calibración
pos_robot_bearing = robotat_get_pose(robotat, no_robot, 'eulzyx')
T_robot = [eul2rotm(deg2rad(pos_robot_bearing(4:6))),[pos_robot_bearing(1);pos_robot_bearing(2);pos_robot_bearing(3)];0,0,0,1];
trplot(T_robot,'frame','A','color','blue','axis',[-3.8 3.8 -4.8 4.8 0 3],'length',0.5)
%% Datos
%definición de goal
goal = 2;       %1 = marker; 0 = posición específica
if (goal == 1)
    pos_goal = robotat_get_pose(robotat, 1, 'eulzyx')*10;
    pos_goal(2) = pos_goal(2)*(-1);
    
else
    pos_goal = [1.5 -1.75]*10;
    %pos_goal = [0 0]*10;
end

%posición inicial
pos_robot = robotat_get_pose(robotat, no_robot, 'eulzyx')*10

% ángulo bearing: 
%bearing = deg2rad(pos_robot_bearing(4));
bearing = deg2rad(-51.9510);

%posición obstaculos

pos_obs12 = robotat_get_pose(robotat, 12, 'eulzyx')*10;
pos_obs14 = robotat_get_pose(robotat, 14, 'eulzyx')*10;
pos_obs16 = robotat_get_pose(robotat, 16, 'eulzyx')*10;
pos_obs18 = robotat_get_pose(robotat, 18, 'eulzyx')*10;
pos_obs19 = robotat_get_pose(robotat, 19, 'eulzyx')*10;
pos_obs20 = robotat_get_pose(robotat, 20, 'eulzyx')*10;
pos_obs21 = robotat_get_pose(robotat, 21, 'eulzyx')*10;
pos_obs22 = robotat_get_pose(robotat, 22, 'eulzyx')*10;

%% Generación mapa
map = logical(zeros(48,38));
x_y_start = [round(abs(pos_robot(1)+38/2))+1,round(abs(pos_robot(2)+48/2))+1];
x_y_goal = [round(abs(pos_goal(1)+38/2))+1,round(abs(pos_goal(2)-48/2))+1];

x_y_obs12 = [round(abs(pos_obs12(1)+38/2))+1,round(abs(pos_obs12(2)-48/2))+1];
x_y_obs14 = [round(abs(pos_obs14(1)+38/2))+1,round(abs(pos_obs14(2)-48/2))+1];
x_y_obs16 = [round(abs(pos_obs16(1)+38/2))+1,round(abs(pos_obs16(2)-48/2))+1];
x_y_obs18 = [round(abs(pos_obs18(1)+38/2))+1,round(abs(pos_obs18(2)-48/2))+1];
x_y_obs19 = [round(abs(pos_obs19(1)+38/2))+1,round(abs(pos_obs19(2)-48/2))+1];
x_y_obs20 = [round(abs(pos_obs20(1)+38/2))+1,round(abs(pos_obs20(2)-48/2))+1];
x_y_obs21 = [round(abs(pos_obs21(1)+38/2))+1,round(abs(pos_obs21(2)-48/2))+1];
x_y_obs22 = [round(abs(pos_obs22(1)+38/2))+1,round(abs(pos_obs22(2)-48/2))+1];


map(x_y_obs12(2),x_y_obs12(1)) = 1;
map(x_y_obs14(2),x_y_obs14(1)) = 1;
map(x_y_obs16(2),x_y_obs16(1)) = 1;

map(x_y_obs18(2),x_y_obs18(1)) = 1;
map(x_y_obs19(2),x_y_obs19(1)) = 1;
map(x_y_obs20(2),x_y_obs20(1)) = 1;
map(x_y_obs21(2),x_y_obs21(1)) = 1;
map(x_y_obs22(2),x_y_obs22(1)) = 1;
figure(1);clf;
map = double(map);
imshow(map);

%% Cálculo de trayectoria
figure(2);clf;
goald = [x_y_goal(1),x_y_goal(2)];
start = [x_y_start(1),x_y_start(2)];
map_trac = flip(map);
ds = Dstar(map_trac, 'inflate', 2);
ds.plan(goald);
ds.plot();
trayectoria = ds.query(start, 'animate');
%% Código

% PID orientación
kpO = 3;
kiO = 0.001; 
kdO = 0;
EO = 0;
eO_1 = 0;

% Acercamiento exponencial
v0 = 2000;
alpha = 30;%0.5;50

xg = pos_goal(1);
yg = pos_goal(2);
thetag = 0;

%Corrección de trayectoria a medidas reales
n = length(trayectoria);
for i=1:n
    trayectoria_r(i,:) = [(trayectoria(i,1)-38/2)/10,(trayectoria(i,2)-48/2)/10];
end


start_x = pos_robot(1);
start_y = pos_robot(2);

goal_x = pos_goal(1);
goal_y = pos_goal(2);

% start_x =  (trayectoria_r(start_x,-5,5,1,500));
% start_y =  (trayectoria_r(start_y,-5,5,1,500));

% goal_x = (trayectoria_r(goal_x,-5,5,1,500));
% goal_y = (trayectoria_r(goal_y,-5,5,1,500));

start_points = [start_x,start_y];
goal = [goal_x,goal_y];

contador = 1;
while true
    pos_robot_bearing = robotat_get_pose(robotat, no_robot, 'eulzyx')
    x = pos_robot_bearing(1)
    y = pos_robot_bearing(2)
    theta = deg2rad(pos_robot_bearing(4))-bearing;
     
   %bearing = bearing * 180 / pi;
   if contador < length(path)
    xg = trayectoria_r(contador,1);
    yg = trayectoria_r(contador,2);
    coords = [xg yg];
    disp(coords)
   end
   
    e = [xg - x; yg - y];
    thetag = atan2(e(2), e(1));
            
    eP = norm(e);
    eO = thetag - theta;
    eO = atan2(sin(eO), cos(eO));
            
    % Control de velocidad lineal
    kP = v0 * (1-exp(-alpha*eP^2)) / eP;
    v = kP*eP;
            
    % Control de velocidad angular
    eO_D = eO - eO_1;
    EO = EO + eO;
    w = kpO*eO + kiO*EO + kdO*eO_D;
    eO_1 = eO;
    
         
    
   giroder = (v + w*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
   giroiz = (v - w*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
   
   robotat_3pi_set_wheel_velocities(robot,giroiz,giroder);
   
   if eP <= 0.25
   contador = contador + 1;
   end
    if (contador == length(trayectoria_r))
        robotat_3pi_force_stop(robot);
        break;
        
    end
    pause(0.1)
    fprintf("en ciclo")
end


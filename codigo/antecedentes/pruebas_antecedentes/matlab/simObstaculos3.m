% =========================================================================
% SIMULACI�N MODELO DIN�MICO CON CONTROL DE FORMACI�N, USANDO COSENO
% HIPERB�LICO, Y EVASI�N DE COLISIONES CON EVASI�N DE OBST�CULOS,
% INCLUYENDO L�MITES DE VELOCIDAD Y CAMBIO DE FORMACI�N (MODIFICACI�N DE
% GR�FICOS Y DETECCI�N DE PUNTOS DE CAMBIO)
% =========================================================================
% Autor: Andrea Maybell Pe�a Echeverr�a
% �ltima modificaci�n: 01/04/2019
% (MODELO 6 con obst�culos)
% =========================================================================
% El siguiente script implementa la simulaci�n del modelo din�mico de
% modificaci�n a la ecuaci�n de consenso utilizando evasi�n de obst�culos y
% luego una combinaci�n de control de formaci�n con una funci�n de coseno 
% hiperb�lico para grafos m�nimamente r�gidos y evasi�n de colisiones.
% Ahora, con evasi�n de obst�culos y cambio de formaci�n, detecci�n de
% puntos de cambio y gr�ficos modificados. 
% =========================================================================

%% Inicializaci�n del mundo
gridsize = 40;      % tama�o del mundo
initsize = 25;
N = 10;             % n�mero de agentes
dt = 0.01;          % per�odo de muestreo
T = 90;             % tiempo final de simulaci�n

% Inicializaci�n de posici�n de agentes
X = initsize*rand(2,N) - initsize/2;
% X(:,1) = [-2,-2]; % posici�n del lider

% Obst�culos
% O = [-8 -5 2; -10 2 -8];  % posiciones obst�culos Set 1
% O = [-7 -5 -7; -12 0 12]; % posiciones obst�culos Set 2
O = [-5 10 -5; -6.5 0 6.5]; % posiciones obst�culos Set 3
cO = size(O,2);             % cantidad de obst�culos
sO = 1;                     % tama�o de los obst�culos 

cW1 = 2;            % contador de agentes sobre agentes
cW2 = 2;            % contador de agentes sobre obst�culos
while(cW1 > 1 || cW2 > 1)
    cW1 = 0;
    cW2 = 0;
    % Asegurar que los agentes no empiecen uno sobre otro
    contR = 1;      % contador de intersecciones
    while(contR > 0)
        contR = 0;
        for i = 1:N
            for j = 1:(N-i)
                resta = norm(X(:,i)-X(:,i+j));  % diferencia entre las posiciones
                if(abs(resta) < 1)
                    X(:,i+j) = X(:,i+j)+[1,1]'; % cambio de posici�n
                    contR = contR+1;            % hay intersecci�n
                end
            end
        end
        cW1 = cW1+1;
    end

    % Asegurar que los agentes no empiecen sobre los obst�culos
    contRO = 1;     % contador de intersecciones con obst�culos
    while(contRO > 0)
        contRO = 0;
        for i = 1:N
            for j = 1:cO
                resta = norm(X(:,i)-O(:,j));    % distancia agente obst�culo
                if(abs(resta) < 3.5)
                    X(:,i) = X(:,i)+[1.8,1.8]'; % cambio de posici�n
                    contRO = contRO+1;          % hay intersecci�n
                end
            end
        end
        cW2 = cW2+1;
    end
end
Xi = X;

% Inicializaci�n de velocidad de agentes
% V = rand(2, N)-0.5; % aleatorio
V = zeros(2, N); % ceros

%% Se grafica la posici�n inicial de los agentes
%  Se utiliza distinci�n de color por grupos de agentes
%    Rojo:    l�der
%    Verde:   agente 1 y agente 2
%    Azul:    agente 3, agente 4 y agente 5
%    Negro:   agente 6, agente 7, agente 8 y agente 9
c = [255 0 0;
     0 255 0;
     0 255 0;
     0 0 255;
     0 0 255;
     0 0 255;
     0 0 0;
     0 0 0;
     0 0 0;
     0 0 0];
hold on;

obstacles = scatter(O(1,:), O(2,:),sO*2000,'filled');
agents = scatter(X(1,:),X(2,:),[], c, 'filled');
grid minor;
hold off;
xlim([-gridsize/2, gridsize/2]);
ylim([-gridsize/2, gridsize/2]);

%% Definici�n e inicializaci�n de las formaciones posibles
Formaciones = {Fmatrix(1,8), Fmatrix(2,8)}; % celda con posiciones posibles
FSel = 1;                                   % formaci�n inicial
Rig = 8;                                    % rigidez de la formaci�n
iniciarCont = false;                        % bandera de conteo cambio
contF = 0;                                  % conteo de ciclos para el cambio
cicloCambio = {};                           % celda para ciclo de cambio
cantCambios = 0;                            % conteo de cambios de formaci�n
puntosCambio = {};                          % celda de puntos de cambio
cantPuntos = 0;                             % cantidad de puntos de cambio
conteoCiclos = 700;                         % ciclos previo al cambio 

%% Selecci�n matriz y par�metros del sistema
d = Fmatrix(FSel,1);    % matriz de formaci�n
r = 1;                  % radio agentes
R = 15;                 % rango sensor
VelMax = 10;            % velocidad m�xima de los agentes

%% Inicializaci�n simulaci�n
t = 0;                      % inicializaci�n de tiempo
ciclos = 1;                 % cuenta de la cantidad de ciclos 
historico = zeros(100*T,N); % hist�rico de velocidades
hX = zeros(100*T,N);        % hist�rico posiciones en X
hY = zeros(100*T,N);        % hist�rico posiciones en Y
Error1 = zeros(1,100*T);    % registro de error formaci�n 1
Error2 = zeros(1,100*T);    % registro de error formaci�n 2
cambio = 0;                 % variable para el cambio de control

while(t < T)
    for i = 1:N
        E = 0;
        for j = 1:N
            dist = X(:,i)- X(:,j); % vector xi - xj
            mdist = norm(dist);    % norma euclidiana vector xi - xj
            dij = 2*d(i,j);        % distancia deseada entre agentes i y j
            
            % Peso a�adido a la ecuaci�n de consenso
            if(mdist == 0 || mdist >= R)
                w = 0;
            else
                switch cambio
                    case 0              % inicio: acercar a los agentes sin chocar
                        w = (mdist - (2*(r + 0.5)))/(mdist - (r + 0.5))^2;
                    case {1,2}
                        if (dij == 0)   % si no hay arista, se usa funci�n "plana" como collision avoidance
                            w = 0.018*sinh(1.8*mdist-8.4)/mdist; 
                        else            % collision avoidance & formation control
                            w = (4*(mdist - dij)*(mdist - r) - 2*(mdist - dij)^2)/(mdist*(mdist - r)^2); 
                        end
                    case 3              % ha llegado a la meta
                        w = 0;
                        cambio = 1;
                end
            end
            % Tensi�n de aristas entre agentes 
            E = E + w.*dist;
        end
        
        %% Collision avoidance con los obst�culos
        for j = 1:cO
            distO = X(:,i)- O(:,j);
            mdistO = norm(distO)-3.5;
            if(abs(mdistO) < 0.0001)
                mdistO = 0.0001;
            end
%             w = ((-mdistO*exp(-mdistO+4))-exp(-mdistO+4))/(mdistO^2); 
            w = -1/(mdistO^2 - 2*mdistO + 1);
            E = E + 0.005*w.*distO;
        end
        
        % Comparaci�n con la velocidad m�xima y ajuste
        if(norm(E) > VelMax)    
            ang = atan2(E(2),E(1));
            E(1) = VelMax*cos(ang);
            E(2) = VelMax*sin(ang);
        end
        % Actualizaci�n de velocidad
        V(:,i) = -1*E;
        
        % Movimiento del l�der
        if (cambio == 2)
            V(:,1) = V(:,1) + 0.05*([-15,0]'-X(:,1));
        end
    end
    
    % Al llegar muy cerca de la posici�n deseada realizar cambio de control
    if(norm(V) < 0.2)
        cambio = cambio + 1;
    end
    % Actualizaci�n de la posici�n de los agentes
    cambio
    norm(V)
    X = X + V*dt;
    
    % Almacenamiento de variables para graficar
    for a = 1:N
        hX(ciclos,a)= X(1,a);
        hY(ciclos,a)= X(2,a);
    end
    historico(ciclos,:) = (sum(V.^2,1)).^0.5;
    
    %% Selecci�n de formaci�n seg�n el error 
    mDist = 0.5*DistEntreAgentes(X);                % distancia actual entre agentes
    Error1(ciclos) = ErrorForm(mDist,Fmatrix(1,8)); % error con formaci�n 1
    Error2(ciclos) = ErrorForm(mDist,Fmatrix(2,8)); % error con formaci�n 2
    FSel_old = FSel;                                % registro de formaci�n anterior
    FSel = SelectForm(Formaciones, mDist);          % selecci�n de formaci�n
    if(FSel ~= FSel_old)
    % cuando la mejor formaci�n ya no es la misma que la actual
        cantPuntos = cantPuntos + 1;            % contador de puntos de cambio
        puntosCambio{cantPuntos} = ciclos;      % almacenamiento puntos de cambio
        iniciarCont = true;                     % iniciar conteo de ciclos
        contF = 0;                              % contador de ciclos
    end
    
    % Conteo de ciclos
    if(iniciarCont)
        contF = contF + 1;
    end
    
    % Hasta que el conteo supere cierta cantidad de ciclos cambiar de
    % formaci�n
    if(contF > conteoCiclos)
        if(iniciarCont == true)
            cantCambios = cantCambios+1;
            cicloCambio{cantCambios} = ciclos*dt;
        end
        d = Fmatrix(FSel,Rig);  % cambio de matriz de formaci�n
        iniciarCont = false;    % reiniciar conteo de ciclos
    end
    
    % Se actualiza la gr�fica, se muestra el movimiento y se incrementa el
    % tiempo
    agents.XData = X(1,:);
    agents.YData = X(2,:);
    pause(dt);
    t = t + dt;
    ciclos = ciclos + 1;
end

%% Gr�ficos

% velocidad - tiempo
figure(1);
plot(0:dt:T-0.01,historico);
xlabel('Tiempo (segundos)');
ylabel('Velocidad (unidades/segundo)');
ylim([-1,inf]);

% trayectorias
figure(2);
hold on;
plot(hX,hY,'--');
xlabel('Posici�n en eje X (unidades)');
ylabel('Posici�n en eje Y (unidades)');
scatter(Xi(1,:),Xi(2,:),[], 'k');
scatter(X(1,:),X(2,:),[], 'k', 'filled');
scatter(O(1,:), O(2,:),sO*3000,'filled');

% error de formaci�n
figure(3);
hold on;
ylim([0,3])
xlabel('Tiempo (segundos)');
ylabel('ECM de la formaci�n (unidades^2)');
e1 = plot(0:dt:T-0.01,Error1,'DisplayName','grafo de prueba 1');
e2 = plot(0:dt:T-0.01,Error2,'DisplayName','grafo de prueba 2');
for i = 1:size(cicloCambio,2)
    line([cicloCambio{i} cicloCambio{i}], [0,Error1(1)],'Color','black','LineStyle','--');
    %scatter(puntosCambio{i}*dt, Error1(puntosCambio{i}));
end
legend([e1 e2],{'grafo de prueba 1', 'grafo de prueba 2'});
hold off;
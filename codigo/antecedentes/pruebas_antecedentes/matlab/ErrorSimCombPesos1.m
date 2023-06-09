% =========================================================================
% MEDICI�N DE M�TRICAS EN LA SIMULACI�N DE COMBINACI�N ADITIVA DEL CONTROL 
% DE FORMACI�N Y EVASI�N DE OBST�CULOS
% =========================================================================
% Autor: Andrea Maybell Pe�a Echeverr�a
% �ltima modificaci�n: 15/07/2019
% (M�tricas MODELO 1)
% =========================================================================
% El siguiente script implementa la simulaci�n de la modificaci�n de la 
% ecuaci�n de consenso al combinar de manera aditiva las modificaciones
% para el control de formaci�n y la evasi�n de obst�culos cierto n�mero de
% veces para determinar las m�tricas de error y c�lculos de energ�a.
% =========================================================================

cantI = 100;                    % cantidad de simulaciones a realizar
EIndividual = zeros(cantI,10);  % energ�a individual por agente en cada simulaci�n
ETotal = zeros(1,cantI);        % energ�a total en cada simulaci�n
EI = zeros(1,cantI);            % error individual en cada simulaci�n
ExitoTotalF = 0;                % cantidad de formaciones 100% exitosas
Exito9F = 0;                    % cantidad de formaciones 90% exitosas
Exito8F = 0;                    % cantidad de formaciones 80% exitosas
Exito7F = 0;                    % cantidad de formaciones 70% exitosas
Fail = 0;                       % cantidad de formaciones fallidas

for I = 1:cantI
    %% Inicializaci�n del mundo
    gridsize = 20;      % tama�o del mundo
    initsize = 20;
    N = 10;             % n�mero de agentes
    dt = 0.01;          % per�odo de muestreo
    T = 20;             % tiempo final de simulaci�n

    % Inicializaci�n de posici�n de agentes
    X = initsize*rand(2,N) - initsize/2;
    X(:,1) = [0,0];     % posici�n del lider
    Xi = X;

    % Inicializaci�n de velocidad de agentes
    % V = rand(2, N)-0.5; % aleatorio
    V = zeros(2, N); % ceros

    %% MATRICES DE ADYACENCIA
    % matrices de adyacencia grafo m�nimamente r�gido
    d1 = [0 1 1 0 0 0 0 0 0 0;
          1 0 1 0 1 1 0 0 0 0;
          1 1 0 1 1 0 0 0 0 0;
          0 0 1 0 1 0 1 1 0 0;
          0 1 1 1 0 1 0 1 1 0;
          0 1 0 0 1 0 0 0 1 1;
          0 0 0 1 0 0 0 1 0 0;
          0 0 0 1 1 0 1 0 0 0;
          0 0 0 0 1 1 0 0 0 1;
          0 0 0 0 0 1 0 0 1 0];

    d2 = [0 1 1 1 0 0 0 0 0 0;
          1 0 1 1 1 1 1 0 0 0;
          1 1 0 0 1 0 0 0 0 0;
          1 1 0 0 0 1 0 0 0 0;
          0 1 1 0 0 0 1 0 1 0;
          0 1 0 1 0 0 0 1 0 0;
          0 1 0 0 1 0 0 1 1 1;
          0 0 0 0 0 1 1 0 0 0;
          0 0 0 0 1 0 1 0 0 1;
          0 0 0 0 0 0 1 0 1 0];

    % matrices de adyacencia todos los nodos conectados
    d2m1 = [0 1 1 1 0 0 0 0 0 0;
            1 0 1 1 1 1 1 0 0 0;
            1 1 0 0 1 0 0 0 0 0;
            1 1 0 0 0 1 0 0 0 0;
            0 1 1 0 0 0 1 0 1 0;
            0 1 0 1 0 0 1 1 0 0;
            0 1 0 0 1 1 0 1 1 1;
            0 0 0 0 0 1 1 0 0 1;
            0 0 0 0 1 0 1 0 0 1;
            0 0 0 0 0 0 1 1 1 0];

    % matrices considerando "segundo grado de adyacencia"
    d2m2 = [0 1 1 1 0 0 2 0 0 0;
            1 0 1 1 1 1 1 0 0 2;
            1 1 0 0 1 2 0 0 2 0;
            1 1 0 0 2 1 0 2 0 0;
            0 1 1 2 0 0 1 0 1 0;
            0 1 2 1 0 0 1 1 2 0;
            2 1 0 0 1 1 0 1 1 1;
            0 0 0 2 2 1 1 0 0 1;
            0 0 2 0 1 2 1 0 0 1;
            0 2 0 0 0 0 1 1 1 0];

    dm1 = [0 1 1 2 0 2 0 0 0 0;
          1 0 1 0 1 1 0 2 0 2;
          1 1 0 1 1 0 2 0 2 0;
          2 0 1 0 1 2 1 1 0 0;
          0 1 1 1 0 1 0 1 1 0;
          2 1 0 2 1 0 0 0 1 1;
          0 0 2 1 0 0 0 1 0 0;
          0 2 0 1 1 0 1 0 0 0;
          0 0 2 0 1 1 0 0 0 1;
          0 2 0 0 0 1 0 0 1 0];

    dm2 = [0 1 1 2 0 2 0 0 0 0;
          1 0 1 0 1 1 0 2 0 2;
          1 1 0 1 1 0 2 0 2 0;
          2 0 1 0 1 2 1 1 0 0;
          0 1 1 1 0 1 0 1 1 0;
          2 1 0 2 1 0 0 0 1 1;
          0 0 2 1 0 0 0 1 0 0;
          0 2 0 1 1 0 1 0 1 0;
          0 0 2 0 1 1 0 1 0 1;
          0 2 0 0 0 1 0 0 1 0];

    % matrices considerando "tercer grado de adyacencia"  
    dm3 = [0 1 1 2 0 2 3 0 0 3;
          1 0 1 0 1 1 0 2 0 2;
          1 1 0 1 1 0 2 0 2 0;
          2 0 1 0 1 2 1 1 0 0;
          0 1 1 1 0 1 0 1 1 0;
          2 1 0 2 1 0 0 0 1 1;
          3 0 2 1 0 0 0 1 2 3;
          0 2 0 1 1 0 1 0 1 2;
          0 0 2 0 1 1 2 1 0 1;
          3 2 0 0 0 1 3 2 1 0];

    % matriz totalmente r�gida tri�ngulo  
    d0 = 2*sqrt(0.75);
    b0 = sqrt((1.5*d0)^2 + 0.25);
    b1 = sqrt(d0^2 + 4);
    b2 = sqrt((0.5*d0)^2 + 2.5^2);

    dr4 = [0 1 1 2 d0 2 3 b0 b0 3;
           1 0 1 d0 1 1 b1 2 d0 2;
           1 1 0 1 1 d0 2 d0 2 b1;
           2 d0 1 0 1 2 1 1 d0 b2;
           d0 1 1 1 0 1 d0 1 1 d0;
           2 1 d0 2 1 0 b2 d0 1 1;
           3 b1 2 1 d0 b2 0 1 2 3;
           b0 2 d0 1 1 d0 1 0 1 2;
           b0 d0 2 d0 1 1 2 1 0 1;
           3 2 b1 b2 d0 1 3 2 1 0];

    d2r = [0 1 1 d0 1 d0 b2 2 3 b2;
           1 0 d0 2 1 1 b1 d0 b2 2;
           1 d0 0 1 1 2 2 d0 b2 b1;
           d0 2 1 0 1 d0 1 1 d0 2;
           1 1 1 1 0 1 d0 1 2 d0;
           d0 1 2 d0 1 0 2 1 d0 1;
           b2 b1 2 1 d0 2 0 1 1 d0;
           2 d0 d0 1 1 1 1 0 1 1;
           3 b2 b2 d0 2 d0 1 1 0 1;
           b2 2 b1 2 d0 1 d0 1 1 0];


    
    %% Inicializaci�n simulaci�n
    t = 0;                      % inicializaci�n de tiempo
    ciclos = 1;                 % cuenta de la cantidad de ciclos 
    historico = zeros(100*T,N); % hist�rico de velocidades

    % Propiedades agentes
    R = 10; % Rango del radar
    r = 1;  % Radio de los agentes

    while(t < T)
        for i = 1:N
            E = 0;
            for j = 1:N
                dist = X(:,i)- X(:,j); % vector xi - xj
                mdist = norm(dist);    % norma euclidiana vector xi - xj
                dij = 2*d2r(i,j);      % distancia deseada entre agentes i y j

                % Peso a�adido a la ecuaci�n de consenso
                if(mdist == 0 || dij == 0)
                    w = [0; 0];
                else
                    w = (mdist - dij).*(dist/mdist);
                end
                w1 = ((2*R - mdist)*dist)/(R - mdist)^2;    % connectivity mantenance
                w2 = ((-2*r + mdist)*dist)/(r - mdist)^2;   % collision avoidance
                % Tensi�n de aristas entre agentes 
                E = E + 5*w + 0.1*w2; 
            end
            % Actualizaci�n de velocidad
            V(:,i) = -0.1*E;
    %         V(:,1) = V(:,1)+0.5; % movimiento del l�der
    %         V(:,1) = 0;          % l�der inm�vil
        end
        % Actualizaci�n de la posici�n de los agentes
        X = X + V*dt;

        % Almacenamiento de variables a evaluar
        historico(ciclos,:) = (sum(V.^2,1)).^0.5;

        % Actualizaci�n de tiempo
        t = t + dt;
        ciclos = ciclos + 1;
    end
    
    %% C�lculo del error final
    mDistF = 0.5*DistEntreAgentes(X);
    errorF = ErrorForm(mDistF,d2r);     % error de formaci�n simulaci�n I
    energiaI = sum(historico.*dt,1);    % energ�a individual simulaci�n I
    energiaT = sum(energiaI,2);         % energ�a total simulaci�n I
    
    EIndividual(I,:) = energiaI;
    ETotal(I) = energiaT;
    EI(I) = errorF;
    
    %% Porcentaje �xito formaci�n
    % Una formaci�n se considera exitosa con un error cuadr�tico medio 
    % menor a 0.05
    if(errorF > 0.05)   
        % Si la formaci�n no fue exitosa se evalua el �xito individual de 
        % de cada agente. Un agente lleg� a la posici�n deseada si tiene un
        % porcentaje de error menor al 15%.
        [errorR,cantAS] = ErrorIndividual(mDistF, d2r, 15);
    else
        % El que la formaci�n haya sido exitosa implica que todos los
        % agentes llegaron a la posici�n deseada
        errorR = errorF;    % error de formaci�n relativo
        cantAS = N;         % cantidad de agentes que llegan a la posici�n deseada
    end
    
    %% Porcentaje de agentes en posici�n deseada
    % Si el error de formaci�n sin tomar en cuenta a los agentes que se
    % alejaron considerablemente, es menor a 0.05 implica que hubo un
    % porcentaje de la formaci�n que s� se logr�. 
    if(errorR < 0.05)
        if(cantAS == N)                     % formaci�n 100% exitosa
            ExitoTotalF = ExitoTotalF + 1;
        elseif (cantAS == round(N*0.9))     % formaci�n 90% exitosa
            Exito9F = Exito9F + 1;
        elseif (cantAS == round(N*0.8))     % formaci�n 80% exitosa     
            Exito8F = Exito8F + 1;
        elseif (cantAS == round(N*0.7))     % formacion 70% exitosa
            Exito7F = Exito7F + 1;
        else                                % formaci�n fallida
            Fail = Fail +1;
        end
    else
        Fail = Fail + 1;
    end
        
    VResults = [ExitoTotalF, Exito9F, Exito8F, Exito7F, Fail];
    
end

% Guardar resultados como resulte m�s conveniente

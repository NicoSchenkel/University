clc; close all; clear all;

% Frame Base an 0 0 0 realtiv zu Worldframe
TWB = eye(4,4);

% Beziehungen der Gelenke zueinander
% TB1: Translation: [0 0 156.4] -q um alte z-achse Rotationen: x:180 
% T12: Translation: [0 5.4 -128.4] | -q um alte y-achse | Rotationen: x:90 y: z:
% T23: Translation: [0 -210.4 -6.4] |q um alte y-achse| Rotationen: x:-90 y: z
% T34: Translation: [0 6.4 -210.4] |-q um alte y-achse |Rotationen: x:90 y: z
% T45: Translation: [0 -208.4 -6.4]  |+q um alte y-achse| Rotationen: x:-90 y: z
% T56: Translation: [0 0 -105.9] | -q um alte y-achse | Rotationen: x:90 y: z
% T67: Translation: [0 -105.9 0]| +q um alte y-achse | Rotationen: x:-90 y: z
% T78: Translation: [0 0 -61.5] | -q = 0, da J8 virtuell | Rotationen: x:180


% Dynamische Rotationen q
q = cell(3,1);
q{1} = transpose([16.39 299.74 5.1 268.15 22.15 63.48 71.2]);
q{2} = transpose([351.73 300.69 8.11 263.98 356.98 63 79.94]);
q{3} = transpose([1.18 291.32 18.47 290.91 94.36 112.93 46]);

% Berechnen der Forward Kinematics
[T08_q1, ToolPosition_q1, Tq1] = ForwardKinematics(TWB, q{1});
[T08_q2, ToolPosition_q2, Tq2] = ForwardKinematics(TWB, q{2});
[T08_q3, ToolPosition_q3, Tq3] = ForwardKinematics(TWB, q{3});

disp("T08 für q1: ");  disp(T08_q1)
disp("Toolposition für q1: ");  disp(ToolPosition_q1)

disp("T08 für q2: ");  disp(T08_q2)
disp("Toolposition für q2: ");  disp(ToolPosition_q2)

disp("T08 für q3");  disp(T08_q3)
disp("Toolposition für q3: ");  disp(ToolPosition_q3)



%% Kinova simulation (Daten)

% Roboter laden
robot = loadrobot('kinovaGen3');
% Set Robot Joint configuration to home configuration
currentRobotJConfig = homeConfiguration(robot);
endEffector = "EndEffector_Link";   % nochmal anschauen!

%% Save Frames for different q's
qFrames_simulation = cell(3,1);

for i = 1:length(q)
    subplot(1,3,i)
    % Initialiseren des Structs, um Roboter mit gewünschten Gelenkwinkeln fahren zu können
    for n = 1:length(q{i})
        % Winkelkonfiguration q an siulation geben, sodass diese an der Endstelle ist
        currentRobotJConfig(n).JointPosition = deg2rad(q{i}(n));
        % End-Wert Position des Endeffektors bekommen
       
    end
     qFrames_simulation{i} = getTransform(robot, currentRobotJConfig, endEffector);
    % Simulaiton anzeigen
    show(robot,currentRobotJConfig, 'Frames','on');

Tq = {Tq1, Tq2, Tq3};   
hold on;                   
drawFrame(Tq{i});
end


%% Ausgabe der Fehler
% Berechnen
fehler1 = (T08_q1 - qFrames_simulation{1});
fehler2 = (T08_q2 - qFrames_simulation{2});
fehler3 = (T08_q3 - qFrames_simulation{3});

% Ausgeben
format short
fehler1 = T08_q1 - qFrames_simulation{1}
fehler2 = T08_q2 - qFrames_simulation{2}
fehler3 = T08_q3 - qFrames_simulation{3}



%% Simulation der Trajektorien

% Tool speed for simulation
timeStep = 0.1; % seconds
toolSpeed = 0.1; % m/s

% Set initial & final end-effector Pose
jointInit = homeConfiguration(robot); % -> Homeposition
taskInit = getTransform(robot,jointInit,endEffector);  % AM ENDE WAHRSCHEINLICH NOCH IN SCHLEIFE PACKEN; DAMIT ROBOTER VON PUNKT A ZU B ZU C FÄHRT UND NICHT IMMER IN DIE HOME POSITION ZURÜCK MUSS

taskFinal = T08_q1;


% Wrap joint movement the qs are into values between -pi and pi (-180° 180°) -> prevent rotations more than 180°


distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));

% Initial & final time
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];


% Using PD-Controler for moving along the trajectorie
jsMotionModel = jointSpaceMotionModel('RigidBodyTree',robot,'MotionType','PDControl');

% Initial States (joint values and velocitites)
qi = currentRobotJConfig.JointPosition; % Initial Position
qdi = zeros(size(qi)); % Initial velocitiy
qddi = zeros(size(qi)); % Initial acceleration
% End Values 
qf = wrapToPi(deg2rad(q{1}));% End Position
qdf = 0; % End velocitiy
qddf = 0; % End acceleration




[rq, rqd, rqdd] = Polynom5DegreeTrajectory(qi, qdi, qddi, qf, qdf, qddf, finalTime, trajTimes);


%% Manipulability


%% Animation der Trajektorie plotten



%% Funciton for Forward Kinematic to calculate Toolposition
function [T08, ToolPosition, T] = ForwardKinematics(TWB, q)
% Entgegennehmen des q- Vektors
Rotationen_q = cell(1, length(q)+1);
% Rotationsmatritzen der einzelnen q's
Rotationen_q {1} = Z_Rotation(-q(1));
Rotationen_q {2} = Y_Rotation(-q(2));
Rotationen_q {3} = Y_Rotation(q(3));
Rotationen_q {4} = Y_Rotation(-q(4));
Rotationen_q {5} = Y_Rotation(q(5));
Rotationen_q {6} = Y_Rotation(-q(6));
Rotationen_q {7} = Y_Rotation(q(7));
Rotationen_q {8} =Z_Rotation(0);
    
% for i=1:length(Rotationen_q)
%     Rotationen_q{i} = rotm2tform(Rotationen_q{i});
% end
  

% Rotationsmatrixen des Roboters (der Frames)
Rotationen_Axis = transpose([
180 0 0;    
90 0 0;
-90 0 0;
90 0 0;
-90 0 0;
90 0 0;
-90 0 0;
180 0 0;
    ]);


% Rotationsmatritzen
% Bereits mit q verrechnet. Translationen werden im zweiten Schritt hinzugefügt. Siehe VL_4 p.5
nFrames = length(Rotationen_Axis);
Rotations_ges = cell(1,nFrames);
    for i = 1:nFrames
        Rotations_ges{i}=  rotm2tform(Rotationen_q{i})  *rotm2tform(Z_Rotation(Rotationen_Axis(3,i)) * Y_Rotation(Rotationen_Axis(2,i)) * X_Rotation(Rotationen_Axis(1,i)));
    end

% Translationen der Gelenke in mm
Translationen = ([ ...
    0, 0, 156.4;
    0, 5.4, -128.4;
    0, -210.4, -6.4;
    0, 6.4, -210.4;
    0, -208.4, -6.4;
    0, 0, -105.9;
    0, - 105.9, 0;
    0, 0, -61.5;
]) * 10^-3;


% Homogenisierte Translationen
Translations_H = cell(1,nFrames);
    for i = 1:nFrames
        Translations_H{i} =   trvec2tform(Translationen(i,:));
    end

% Transformationen berechnen
Transformationen = cell(1,nFrames);
    for i=1:nFrames
        Transformationen{i} = Translations_H{i}  * Rotations_ges{i};
    end


% Transformationen von Frame zu Frame berechnen in abhängigkeit von vorhigen Frames
T = cell(1, length(Transformationen));
for i=1: length(Transformationen)
    if i == 1
    T{i} = TWB * Transformationen{i};
    else
        T{i} = T{i-1} * Transformationen{i};
    end
end

T08 = T{8};

% Tool
v = transpose([0 0 0.12 1]);
ToolPosition = T{8} * v;

end





%% Funktionen für Rotationsmatrizen
% Rotationssmatrix X
function [Rx] = X_Rotation(alpha)
    
    Rx = [1,               0,                 0;
          0,     cosd(alpha),     -sind(alpha) ;
          0,     sind(alpha),      cosd(alpha)];
end

% Rotationssmatrix Y
function [Ry] = Y_Rotation(beta)
    
    Ry = [cosd(beta),  0,  sind(beta);
                   0,  1,            0;
         -sind(beta),  0,  cosd(beta)];
end

% Rotationssmatrix Z
function [Rz] = Z_Rotation(gamma)
    Rz = [cosd(gamma), -sind(gamma),  0;
          sind(gamma),  cosd(gamma),  0;
              0,            0,        1];
end




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
% T78: Translation: [0 0 -61.5] | +q = 0, da J8 virtuell | Rotationen: x:180


% Dynamische Rotationen q
q = cell(3,1);
q{1} = transpose([16.39 299.74 5.1 268.15 22.15 63.48 71.2]);
q{2} = transpose([351.73 300.69 8.11 263.98 356.98 63 79.94]);
q{3} = transpose([1.18 291.32 18.47 290.91 94.36 112.93 46]);

% Berechnen der Forward Kinematics
[T08_q1, ToolPosition_q1] = ForwardKinematics(TWB, q{1});
[T08_q2, ToolPosition_q2] = ForwardKinematics(TWB, q{2});
[T08_q3, ToolPosition_q3] = ForwardKinematics(TWB, q{3});

disp("T08 für q1: ");  disp(T08_q1)
disp("Toolposition für q1: ");  disp(ToolPosition_q1)

disp("T08 für q2: ");  disp(T08_q2)
disp("Toolposition für q2: ");  disp(ToolPosition_q2)

disp("T08 für q3");  disp(T08_q3)
disp("Toolposition für q3: ");  disp(ToolPosition_q3)

%% Kinova simulation


% Roboter laden
robot = loadrobot('kinovaGen3');
% Set Robot JOint configuration to home configuration
currentRobotJConfig = homeConfiguration(robot);
endEffector = "EndEffector_Link";   % nochmal anschauen!

% Tool speed for simulation
timeStep = 0.1; % seconds
toolSpeed = 0.1; % m/s

% Set initial Pose
jointInit = currentRobotJConfig; % -> Homeposition
taskInit = getTransform(robot,jointInit,endEffector);

% Setting the final end-effector pose
taskFinal = cell(3,1)
taskFinal{1} = T08_q1;
taskFinal{2} = T08_q2;
taskFinal{3} = T08_q3;

% Compute travel distance
distance = cell(3,1)
for n=1:length(taskFinal)
distance{n} = norm(tform2trvec(taskInit)-tform2trvec(taskFinal{n})); % Translationen werden aus den Translationsmatrixen extrahiert; durch norm wir duch den euklidischen Satz die Distanz berechnet
end

% Zeit, die für die Trajektorie benötigt wird durch Distanz und ToolSpeed berechnen
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];

% Interpolate waypoints
[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); % Quaternion wird verwendet, da man damit die Distanzen der Winkel berechen kann ohne auf Sinularität zu treffen

% Modelierung die Bewegung des Roboters im kartesischen Raum durch PD-Control (folgen der Wegpunkte ohne Abweichunge
% -Positionsfehler) 
tsMotionModel = taskSpaceMotionModel('RigidBodyTree',robot,'EndEffectorName','EndEffector_Link');

% D und P auf Null setzen, sodass nur der Trajektorie gefolgt wird; Orientierung wird vernachlässigt
tsMotionModel.Kp(1:3,1:3) = 0;
tsMotionModel.Kd(1:3,1:3) = 0;

% Definieren der Anfangsbedingungen (Position und Geschwindigkeit)
q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));


%% Anschauen!!!
[tTask,stateTask] = ode15s(@(t,state) exampleHelperTimeBasedTaskInputs(tsMotionModel,timeInterval,taskInit,taskFinal,t,state),timeInterval,[q0; qd0]);



% Save Frames for different q's
qFrames = cell(3,1);
for i = 1:length(q)
    subplot(1,3,i)
    config = randomConfiguration(robot); % Initialiseren des Structs, um Roboter mit gewünschten Gelenkwinkeln fahren zu können
    for n = 1:length(q{i})
        % Winkelkonfiguration q an siulation geben, sodass diese an der Endstelle ist
        config(n).JointPosition = deg2rad(q{i}(n));
        % End-Wert Position des Endeffektors bekommen
       
    end
     qFrames{i} = getTransform(robot, config, endEffector);
    % Simulaiton anzeigen
    show(robot,config);
end


%% Ausgabe der Fehler
% Berechnen
fehler1 = (T08_q1 - qFrames{1});
fehler2 = (T08_q2 - qFrames{2});
fehler3 = (T08_q3 - qFrames{3});
% Ausgeben
format short
fehler1 = T08_q1 - qFrames{1}
fehler2 = T08_q2 - qFrames{2}
fehler3 = T08_q3 - qFrames{3}


%% Manipulability


%% Funciton for Forward Kinematic to calculate Toolposition
function [T08, ToolPosition] = ForwardKinematics(TWB, q)

Rotationen_q = cell(1, length(q)+1);
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
  

% Rotationen
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

% Translationen der Gelenke
Translationen = ([ ...
    0,   0,      156.4;
    0,   5.4,   -128.4;
    0,   -210.4,  -6.4;
    0,    6.4,    -210.4;
    0, -208.4,    -6.4;
    0,   0,   -105.9;
    0, - 105.9,   0;
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
function [Rx] = X_Rotation(alpha)
    
    Rx = [1,          0,                      0;
              0,     cosd(alpha), -sind(alpha) ;
              0,     sind(alpha),  cosd(alpha)];
    %Rx = rotm2tform(Rx);
end

function [Ry] = Y_Rotation(beta)
    
    Ry = [cosd(beta),  0,  sind(beta);
                   0,      1,       0;
             -sind(beta),  0,  cosd(beta)];
    %Ry = rotm2tform(Ry);
end


function [Rz] = Z_Rotation(gamma)
    Rz = [cosd(gamma), -sind(gamma),  0;
              sind(gamma),  cosd(gamma),  0;
                   0,            0,       1];
   % Rz = rotm2tform(Rz);
end




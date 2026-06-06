clc; close all; clear all;

TWB = eye(4,4);

q = cell(3,1);
q{1} = transpose([16.39 299.74 5.1 268.15 22.15 63.48 71.2]);
q{2} = transpose([351.73 300.69 8.11 263.98 356.98 63 79.94]);
q{3} = transpose([1.18 291.32 18.47 290.91 94.36 112.93 46]);

% Forward Kinematics (jetzt T{9} als Tool-Frame)
[T09_q1, ToolPosition_q1, Transformations_q1] = ForwardKinematics(TWB, q{1});
[T09_q2, ToolPosition_q2, Transformations_q2] = ForwardKinematics(TWB, q{2});
[T09_q3, ToolPosition_q3, Transformations_q3] = ForwardKinematics(TWB, q{3});

disp("T09 für q1: ");  disp(T09_q1)
disp("Toolposition für q1: ");  disp(ToolPosition_q1)
disp("T09 für q2: ");  disp(T09_q2)
disp("Toolposition für q2: ");  disp(ToolPosition_q2)
disp("T09 für q3: ");  disp(T09_q3)
disp("Toolposition für q3: ");  disp(ToolPosition_q3)


%% Roboter + Gripper laden
robot   = loadrobot('kinovaGen3');
gripper = loadrobot('robotiq2F85');
addSubtree(robot, 'EndEffector_Link', gripper);  % ← Gripper anhängen

endEffector = "EndEffector_Link";          % ← neuer Tip-Link
toolOffset  = trvec2tform([0 0 0.12]);

figure;
show(robot, homeConfiguration(robot), 'Frames', 'off');

currentRobotJConfig = homeConfiguration(robot);


%% Frames für verschiedene q's
qFrames_simulation = cell(3,1);
Transformations_ = {Transformations_q1, Transformations_q2, Transformations_q3};

for i = 1:3
    subplot(1,3,i);
    for n = 1:7   % nur die 7 Roboter-Gelenke, nicht die Gripper-Gelenke
        currentRobotJConfig(n).JointPosition = deg2rad(q{i}(n));
    end
    qFrames_simulation{i} = getTransform(robot, currentRobotJConfig, endEffector) * toolOffset;
    show(robot, currentRobotJConfig, 'Frames', 'on');
    hold on;
    drawFrame(Transformations_{i});
end


%% Fehlerausgabe
format short
fehler1 = T09_q1 - qFrames_simulation{1}
fehler2 = T09_q2 - qFrames_simulation{2}
fehler3 = T09_q3 - qFrames_simulation{3}


%% Trajektorien
targets = {T09_q1, T09_q2, T09_q3};  % ← T09 statt T08

figure2 = figure('Name', 'Trajektorienbewegung');
axAnim  = axes(figure2);

configNew = homeConfiguration(robot);

for i = 1:length(targets)
    jointInit = configNew;
    taskInit = getTransform(robot, jointInit, endEffector) * toolOffset;
    taskFinal = targets{i};

    [finalTime, trajTimes] = calculateTrajectoryParams(taskInit, taskFinal);

    qi   = wrapToPi([jointInit.JointPosition]');
    qdi  = zeros(size(qi));
    qddi = zeros(size(qi));

    qf_full = wrapToPi([jointInit.JointPosition]');  % ← Gripper-Gelenke bleiben wo sie sind
    qf_full(1:7) = wrapToPi(deg2rad(q{i}));          % ← nur Roboter-Gelenke setzen
    qdf  = zeros(size(qf_full));
    qddf = zeros(size(qf_full));

    [rq, ~, ~] = Polynom5DegreeTrajectory(qi, qdi, qddi, qf_full, qdf, qddf, finalTime, trajTimes);

    configNew = drawTrajectoryGripper(trajTimes, rq, robot, configNew, endEffector, axAnim, toolOffset);
end


%% Forward Kinematics Funktion (jetzt mit T{9})
function [T09, ToolPosition, T] = ForwardKinematics(TWB, q)

Rotationen_q = cell(1, length(q)+1);
Rotationen_q{1} = Z_Rotation(-q(1));
Rotationen_q{2} = Y_Rotation(-q(2));
Rotationen_q{3} = Y_Rotation(q(3));
Rotationen_q{4} = Y_Rotation(-q(4));
Rotationen_q{5} = Y_Rotation(q(5));
Rotationen_q{6} = Y_Rotation(-q(6));
Rotationen_q{7} = Y_Rotation(q(7));
Rotationen_q{8} = Z_Rotation(0);

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

nFrames = length(Rotationen_Axis);
Rotations_ges = cell(1, nFrames);
for i = 1:nFrames
    Rotations_ges{i} = rotm2tform(Rotationen_q{i}) * rotm2tform( ...
        Z_Rotation(Rotationen_Axis(3,i)) * ...
        Y_Rotation(Rotationen_Axis(2,i)) * ...
        X_Rotation(Rotationen_Axis(1,i)));
end

Translationen = ([
    0,    0,     156.4;
    0,    5.4,  -128.4;
    0, -210.4,    -6.4;
    0,    6.4,  -210.4;
    0, -208.4,    -6.4;
    0,    0,    -105.9;
    0, -105.9,     0;
    0,    0,     -61.5;
]) * 1e-3;

Translations_H = cell(1, nFrames);
for i = 1:nFrames
    Translations_H{i} = trvec2tform(Translationen(i,:));
end

Transformationen = cell(1, nFrames);
for i = 1:nFrames
    Transformationen{i} = Translations_H{i} * Rotations_ges{i};
end

T = cell(1, nFrames + 1);   % ← +1 für T{9}
for i = 1:nFrames
    if i == 1
        T{i} = TWB * Transformationen{i};
    else
        T{i} = T{i-1} * Transformationen{i};
    end
end

% T{9}: Tool-Frame mit Offset vom Gripper (Abstand EndEffector zu Fingerpads)
toolOffset = trvec2tform([0 0 0.12]);   % ← ggf. anpassen
T{9}  = T{8} * toolOffset;             % ← neu
T09   = T{9};                           % ← Rückgabe

ToolPosition = T{9}(1:3, 4);           % ← direkt aus T{9} statt v-Vektor
end


%% Rotationsfunktionen
function Rx = X_Rotation(alpha)
    Rx = [1,           0,            0;
          0, cosd(alpha), -sind(alpha);
          0, sind(alpha),  cosd(alpha)];
end

function Ry = Y_Rotation(beta)
    Ry = [cosd(beta), 0, sind(beta);
                   0, 1,          0;
         -sind(beta), 0, cosd(beta)];
end

function Rz = Z_Rotation(gamma)
    Rz = [cosd(gamma), -sind(gamma), 0;
          sind(gamma),  cosd(gamma), 0;
                    0,            0, 1];
end
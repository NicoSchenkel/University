clc; close all; clear all;

% Frame Base an 0 0 0 relativ zu Worldframe
TWB = eye(4,4);

% Dynamische Rotationen q
q = cell(3,1);
q{1} = transpose([16.39 299.74 5.1 268.15 22.15 63.48 71.2]);
q{2} = transpose([351.73 300.69 8.11 263.98 356.98 63 79.94]);
q{3} = transpose([1.18 291.32 18.47 290.91 94.36 112.93 46]);

% Berechnen der Forward Kinematics
[T08_q1, ToolPosition_q1, ~, T_all_q1] = ForwardKinematics(TWB, q{1});
[T08_q2, ToolPosition_q2, ~, T_all_q2] = ForwardKinematics(TWB, q{2});
[T08_q3, ToolPosition_q3, ~, T_all_q3] = ForwardKinematics(TWB, q{3});

disp("T08 für q1: ");  disp(T08_q1)
disp("Toolposition für q1: ");  disp(ToolPosition_q1)
disp("T08 für q2: ");  disp(T08_q2)
disp("Toolposition für q2: ");  disp(ToolPosition_q2)
disp("T08 für q3: ");  disp(T08_q3)
disp("Toolposition für q3: ");  disp(ToolPosition_q3)

%% Kinova Simulation
robot = loadrobot('kinovaGen3');
endEffector = "EndEffector_Link";

%% Farbdefinitionen für berechnete Frames
% X-Achse = Cyan   [0   0.8 0.8]
% Y-Achse = Magenta [0.8 0   0.8]
% Z-Achse = Orange  [1   0.5 0  ]
ownColors = {[0 0.8 0.8], [0.8 0 0.8], [1 0.5 0]};

%% Für jedes q: Roboter anzeigen + beide Frame-Typen einzeichnen
T_all_cells = {T_all_q1, T_all_q2, T_all_q3};
qFrames = cell(3,1);

for i = 1:length(q)
    figure;

    % Gelenkkonfiguration setzen
    config = randomConfiguration(robot);
    for n = 1:length(q{i})
        config(n).JointPosition = deg2rad(q{i}(n));
    end

    % Simulationsframe des EndEffektors holen
    qFrames{i} = getTransform(robot, config, endEffector);

    % Roboter anzeigen
    show(robot, config, 'PreservePlot', false);
    hold on;
    title(sprintf('Roboter mit Frames für q%d', i));

    % --- Berechnete Frames T01..T08 einzeichnen (Cyan / Magenta / Orange) ---
    drawOwnFrames(T_all_cells{i}, 0.05, ownColors);

    % --- Simulationsframe des EndEffektors einzeichnen (schwarz gestrichelt) ---
    drawSimFrame(qFrames{i}, 0.07);

    % Legende
    h1 = plot3(nan,nan,nan, '-',  'Color', ownColors{1}, 'LineWidth', 2);
    h2 = plot3(nan,nan,nan, '-',  'Color', ownColors{2}, 'LineWidth', 2);
    h3 = plot3(nan,nan,nan, '-',  'Color', ownColors{3}, 'LineWidth', 2);
    h4 = plot3(nan,nan,nan, 'k--','LineWidth', 2);
    legend([h1 h2 h3 h4], ...
        'X-Achse berechnet  (Cyan)', ...
        'Y-Achse berechnet  (Magenta)', ...
        'Z-Achse berechnet  (Orange)', ...
        'EndEffector Simulation (Schwarz)', ...
        'Location', 'best');
end

%% Fehlerberechnung
format short
fehler1 = T08_q1 - qFrames{1}
fehler2 = T08_q2 - qFrames{2}
fehler3 = T08_q3 - qFrames{3}


%% ===================== FUNKTIONEN =====================

%% Berechnete Frames T01..T08 einzeichnen (eigene Farben: Cyan / Magenta / Orange)
function drawOwnFrames(T_all, scale, colors)
    for i = 1:length(T_all)
        Frame  = T_all{i};
        origin = Frame(1:3, 4);

        for ax = 1:3
            dir      = Frame(1:3, ax);
            endpoint = origin + scale * dir;
            plot3([origin(1), endpoint(1)], ...
                  [origin(2), endpoint(2)], ...
                  [origin(3), endpoint(3)], ...
                  '-', 'Color', colors{ax}, 'LineWidth', 6);
        end

        % Ursprung als kleiner schwarzer Punkt
        plot3(origin(1), origin(2), origin(3), ...
              'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 3);

        % Frame-Beschriftung
        text(origin(1), origin(2), origin(3), ...
             sprintf('  T_{0%d}', i), ...
             'FontWeight', 'bold', 'FontSize', 8, 'Color', [0.2 0.2 0.2]);
    end
end

%% Simulationsframe des EndEffektors einzeichnen (schwarz gestrichelt)
function drawSimFrame(T, scale)
    origin = T(1:3, 4);
    for ax = 1:3
        dir      = T(1:3, ax);
        endpoint = origin + scale * dir;
        plot3([origin(1), endpoint(1)], ...
              [origin(2), endpoint(2)], ...
              [origin(3), endpoint(3)], ...
              'k--', 'LineWidth', 6);
    end
    text(origin(1), origin(2), origin(3), ...
         '  T_{08,sim}', 'FontWeight', 'bold', ...
         'FontSize', 9, 'Color', 'k');
end


%% Forward Kinematics
function [T08, ToolPosition, Rotations_ges, T_all] = ForwardKinematics(TWB, q)

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
        Rotations_ges{i} = rotm2tform(Rotationen_q{i}) * ...
                           rotm2tform(Z_Rotation(Rotationen_Axis(3,i)) * ...
                                      Y_Rotation(Rotationen_Axis(2,i)) * ...
                                      X_Rotation(Rotationen_Axis(1,i)));
    end

    Translationen = ([
        0,      0,    156.4;
        0,    5.4,   -128.4;
        0, -210.4,     -6.4;
        0,    6.4,   -210.4;
        0, -208.4,     -6.4;
        0,      0,   -105.9;
        0, -105.9,      0.0;
        0,      0,    -61.5;
    ]) * 1e-3;

    Translations_H = cell(1, nFrames);
    for i = 1:nFrames
        Translations_H{i} = trvec2tform(Translationen(i,:));
    end

    Transformationen = cell(1, nFrames);
    for i = 1:nFrames
        Transformationen{i} = Translations_H{i} * Rotations_ges{i};
    end

    T = cell(1, nFrames);
    for i = 1:nFrames
        if i == 1
            T{i} = TWB * Transformationen{i};
        else
            T{i} = T{i-1} * Transformationen{i};
        end
    end

    T_all        = T;
    T08          = T{8};
    v            = transpose([0 0 0.12 1]);
    ToolPosition = T{8} * v;
end


%% Rotationsmatrizen
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
                    0,           0,  1];
end
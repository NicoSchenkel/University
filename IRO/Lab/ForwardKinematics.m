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




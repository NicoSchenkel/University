clc; close all; clear;

% Erstellen der Vektoren
o0 = transpose([0 0 0]);
% Vektor in X-Richtung
a0 = transpose([1 0 0]);
% Vektor in Y-Richtung
b0 = transpose([0 1 0]);
% Vektor in Z-Richtung
c0 = transpose([0 0 1]);

% Vektoren zum Anzeigen des Frames
x = [o0(1) a0(1)];
y = [o0(2) b0(2)];
z = [o0(3) c0(3)];
zero = [0, 0];

%Plotten des Origin Frames
plot3(x,zero,zero, "red")
hold on;
plot3(zero,y,zero, "green")
plot3(zero,zero,z, "blue")




% Rotations Winkel der Frames 
alpha = [-89 -89 91];
beta = [-175 -175 -5]
gamma = [90 90 10];
% Rotationsmatrix 
% Order because premultiply matrices are requiered 
Rot1 =  Z_Rotation(gamma(1)) * Y_Rotation(beta(1)) * X_Rotation(alpha(1))

Rot
% Translations
o0o1 = ([-0.78 0.15 0.21]);
o0o2 = transpose([-0.78 -0.05 0.21]);
o0o3 = transpose([-0.55 -0.2 0.21]);




%drawFrame(o0, Rot1)




function drawFrame(O, R)

end


function [Rx] = X_Rotation(alpha)
    
    Rx = [1,          0,           0;
              0,     cosd(alpha), -sind(alpha);
              0,     sind(alpha),  cosd(alpha)];
%    Rx = rotm2tform(Rx);
end

function [Ry] = Y_Rotation(beta)
    
    Ry = [cosd(beta),  0,  sind(beta);
                   0,      1,       0;
             -sind(beta),  0,  cosd(beta)];
%    Ry = rotm2tform(Ry);
end


function [Rz] = Z_Rotation(gamma)
    Rz = [cosd(gamma), -sind(gamma),  0;
              sind(gamma),  cosd(gamma),  0;
                   0,            0,       1];
%    Rz = rotm2tform(Rz);
end




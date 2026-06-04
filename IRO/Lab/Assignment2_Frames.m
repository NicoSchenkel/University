clc; close all; clear;

% Erstellen der Vektoren
O0 = transpose([0 0 0]);
% Vektor in X-Richtung
A0 = transpose([1 0 0]);
% Vektor in Y-Richtung
B0 = transpose([0 1 0]);
% Vektor in Z-Richtung
C0 = transpose([0 0 1]);
Frame0 = [A0 B0 C0];

% Plotten des Origin Frames
% Plot 3 zeichnet Vektor von dem Start zu Endpunkt
    % X-Vektor/Axe
    plot3([O0(1) A0(1)], [O0(1) A0(2)], [O0(1) A0(3)], "red");
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
    hold on;
    % Y-Vektor/Axe
    plot3([O0(1) B0(1)], [O0(1) B0(2)], [O0(1) B0(3)], "green");
    % Z-Vektor/Axe
    plot3([O0(1) C0(1)], [O0(1) C0(2)], [O0(1) C0(3)], "blue");

% Rotations Winkel der Frames 
    alpha = [-89 -89 91];
    beta = [-175 -175 -5];
    gamma = [90 90 10];

    nFrames = length(alpha);
% Rotationsmatrix   
    R_ges = cell(1,nFrames);
    for i = 1:nFrames
        % Erstellen der Matrix mit Rotation und Origin
        % Order because premultiply matrices are requiered
        R_ges{i}=  rotm2tform(Z_Rotation(gamma(i)) * Y_Rotation(beta(i)) * X_Rotation(alpha(i)));
    end

    % Rotationsmatrix, die Origin enthält
     R_ges1 = R_ges{1};
  

     % Translationen
O0O1 = transpose([-0.78 0.15 0.21]);
O0O2 = transpose([-0.78 -0.05 0.21]);
O0O3 = transpose([-0.55 -0.2 .21]);
TranslationVectors = [O0O1 O0O2 O0O3];
  
% Frames Zeichnen 
Frames = drawFrame(TranslationVectors, R_ges); 


%% Spatial Transformations between Frames %%%%%%%%%%%%%%%%%%%%%%%%
Frame01 =Frames{1};
T01 = Frames{1};
 
%T21 = inv(Frames{2}) * Frames{1};
T21 = Frames{2} \ Frames{1}; 
%T32 = inv(Frames{3})  * Frames{2};
T32 = (Frames{3})  \ Frames{2}; % Faster then calculate with the inverse
T20 = inv(Frames{2});


Transforms = {T21, T32, T20};
labels     = {'T21', 'T32', 'T20'};

for i = 1:length(Transforms)
    T = Transforms{i};
    fprintf('\n--- %s ---\n', labels{i});
    for row = 1:4
        fprintf(sprintf('%8.4f  %8.4f  %8.4f  %8.4f\n', ...
            T(row,1), T(row,2), T(row,3), T(row,4)));
    end
end

%% Transformation of Point Coordinates
P3 = transpose([1 0 0 1]);
P2 = transpose([-1 0 -1 1]);
 
Frame0 = rotm2tform(Frame0);
Frame3 = Frames{3};

% P3
P3F0 = Frames{3} * P3
plot3(P3F0(1), P3F0(2), P3F0(3), 'co', LineWidth=2)
text(P3F0(1), P3F0(2), P3F0(3), '   P3');
% P2
P2F0 = Frames{2} * P2
plot3(P2F0(1), P2F0(2), P2F0(3), 'co', LineWidth= 2)
text(P2F0(1), P2F0(2), P2F0(3), '   P2');

 
%% Functions
function Frames = drawFrame(O, R)

    % Translation in Homogene Matrix
    T = cell(1,size(O, 2));
    for i=1: size(O, 2)
        T{i} = trvec2tform(transpose(O(:,i)));
    end
    % Erstellen der Frames
    Frames = cell(1,length(T));
    for i=1:length(T)
    Frames{i} =  T{i} * R{i};
    end
    
    % Frames
    for i = 1:length(Frames)
        Frame = Frames{i};
        % Beim Plotten darauf achten, dass die Frames die Verschiebung des
        % Vektors zurückgeben, nicht die absolute Position. Daher muss der Origin des neuen Frames noch draufgerechnet werden !
        % X-Achse
        plot3( [Frame(1,4) Frame(1,4)+Frame(1,1)], [Frame(2,4) Frame(2,4)+Frame(2,1)], [Frame(3,4) Frame(3,4)+Frame(3,1)], 'r')
        % Y-Achse
        plot3( [Frame(1,4) Frame(1,4)+Frame(1,2)], [Frame(2,4) Frame(2,4)+Frame(2,2)], [Frame(3,4), Frame(3,4)+Frame(3,2)], 'g')
        % Z-Achse 
        plot3( [Frame(1,4) Frame(1,4)+Frame(1,3)], [Frame(2,4) Frame(2,4)+Frame(2,3)], [Frame(3,4) Frame(3,4)+Frame(3,3)], 'b')
        text(Frame(1,4), Frame(2,4), Frame(3,4), sprintf(' F_%d', i), 'FontWeight', 'bold');
    
        % xVektor = (Frame(1:3,1))
        % yVektor = (Frame(1:3,2))
        % zVektor = (Frame(1:3,3))
        % 
        % lenghts = [norm(xVektor) norm(yVektor) norm(zVektor)]
        % orth = [dot(xVektor,yVektor) dot(xVektor,zVektor) dot(zVektor,yVektor)]
    
        
    end
    
    
    % Wichtig für die korrekte Anzeige der Proportionen
    axis equal; 
    grid on;
    view(3);
end





function [Rx] = X_Rotation(alpha)
    
    Rx = [1,          0,                      0;
              0,     cosd(alpha), -sind(alpha) ;
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
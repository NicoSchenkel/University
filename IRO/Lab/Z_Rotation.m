function [Rz] = Z_Rotation(gamma)
    Rz = [cosd(gamma), -sind(gamma),  0;
          sind(gamma),  cosd(gamma),  0;
              0,            0,        1];
end
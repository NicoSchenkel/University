
function [J] = jacobiMatrix(T, TWB)
    position_endeffektor = T{end}(1:3,4);
    % prelocating size for J for more speed
    num_joints = 7;
    Jv = zeros(3, num_joints);
    Jw = zeros(3, num_joints);
    
    % Exctract each T0nj|nj = number of joints|Hence tool is rigid it doesnt have rotation
    for n=1:num_joints

        if n == 1
            prevRot = TWB(1:3,3);
            prev_JointLocation = TWB(1:3,4);
        
        else
        	prevRot = T{n-1}(1:3,3);
            prev_JointLocation = T{n-1}(1:3,4);
        end
            
            % v = w x r - durch multiplikation mit qdot wird rotation zu w
            % Da Matlab von 1 startet immer n-1
            Jv(:,n) = cross(prevRot, position_endeffektor - prev_JointLocation);
            Jw(:,n) = prevRot;
        
    end
J = [Jv; Jw];

E = J * J';
syms lambda

end


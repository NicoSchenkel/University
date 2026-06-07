function [vm_min, vm_max] = manipulability(J)
%% Manipulability
Jv = J(4:6,:);
Ev = Jv * Jv';

% D - Diagonlae Matrix der Eigenwerte (mainpulabilität)
% |V - Spalten entsprechen den jeweiligen Eigenvektoren (Richtung der Manipulabilität)
[V,D] = eig(Ev);
% Direction of main axis of e are the eigenvectors V
% , while the lenght of the axes are described by the sqrt of the corresponding eigenvalue D
eigenwerte = diag(D); % Um Skalierung zu

% Prüfen welcher Eigenwert am größten ist - größte Manipulierbarkeit
[lambda_max, idx_max] = max(eigenwerte);
% Prüfen welcher Eigenwert am kleinsten ist - geringste Manipulierbarkeit
[lambda_min, idx_min] = min(eigenwerte);
% Richtung bestimmen der Manipulierbarkeitn und dann skalieren
vm_max = V(:,idx_max) * sqrt(lambda_max);
vm_min = V(:,idx_min) * sqrt(lambda_min);
end
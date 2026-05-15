clc; clear all; close all;
%% Aufgabe 1
% Gegeben:
alpha = 30; %degree
t = linspace(0,50);
% DGL Pendel:
% phi_2dot + sin(alpha) * g/l = 0
% phi_2dot = -sind(aplha * g/l
% Anfangsbedingungen (nicht gegeben daher:
y = [alpha, 0];

[t,y] = ode23(@f,t,y);
% Weg
s = y(:,1);
% Geschwindigkeit
v = y(:, 2);

subplot(3,1,1)
plot(t,s)
grid on
xlabel("t [s]")
ylabel("s [alpha]")
title("Weg-Zeit Diagramm")

subplot(3,1,2)
plot(t,v, 'r')
grid on
xlabel("Zeit")
ylabel("v[rad/s]")
title("Geschwindigkeits-Zeit Diagramm")

subplot(3,1,3)
plot(t,v, 'r', t,s, 'b')
grid on
hold on
yline(0, 'k', 'LineWidth', 1.5); % Verstärkte Nulllinie auf der X-Achse
title("Geschwindigkeits-Zeit Diagramm")



%% Aufgabe 2

k = 4;
w = 0.5; 
t = linspace(0,50);

phi = transpose(linspace(0, 2*pi));
% Darstellung der Kurve
x =  k .* (1- cos(phi)) .* cos(phi); 
y =  k .* (1- cos(phi)) .* sin(phi);

% Geschwindigkeiten 
vx = k .* sin(w.*t) .* (2.*cos(w.*t)-1);
vy = k .* (sin(w.*t).^2 + cos(w.*t) - cos(w.*t).^2);
betrag = sqrt(vx.^2 + vy.^2);

figure(1)
plot(x,y)
title("Darstellung der Kurve")

figure(2)
subplot(3,1,1)
plot(t, vx)
title("vx")
xlabel("t[s]")
ylabel("vx[m/s]")
grid on

subplot(3,1,2)
plot(t, vy)
title("vy")
xlabel("t[s]")
ylabel("vy[m/s]")
grid on

subplot(3,1,3)
plot(t, betrag)
title("|v|")
xlabel("t[s]")
ylabel("v[m/s]")
grid on






%% Functions
function dydt = f(t,y)
% Pendel maße:
l = 1;    % m
m = 2;    % kg
g = 9.81; % Graviattionskraft
dydt = zeros(2,1);
% Ableitung des Winkels = Winkelgeschwindigkiet
dydt(1) =  y(2);
% Ableitung der Geschwindigkeit = Winkelbeschleunigung
dydt(2) = -sind(y(1)) * (g/l);

end

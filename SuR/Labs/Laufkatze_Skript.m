clc; clear; close all
sim("Laufkatze.slx")
t       = ans.tout;
x       = ans.x.Data;
v   = ans.x_dot.Data;
p     = ans.phi.Data;
p_dot = ans.phi_dot.Data;
t_60 = find(x >=60 , 1);
x_t60= x(t_60);
figure;

subplot(2,2,1); plot(t, x);            title('Position x');          ylabel('x[m]');     xlabel('t [s]'); ylim([0 70]); grid on
hold on;
% Wir nehmen den Zeitstempel an dem der Index t_60 erreicht wird
zeitpunkt = t(t_60); 
xline(zeitpunkt, '--','Color', 'r', 'LineWidth', 0.5); 
hold off;

subplot(2,2,2); plot(t, v);        title('Geschwindigkeit ẋ');   ylabel('v[m/s]');   xlabel('t [s]'); grid on
hold on;
% Wir nehmen den Zeitstempel an dem der Index t_60 erreicht wird
zeitpunkt = t(t_60); 
xline(zeitpunkt, '--','Color', 'r', 'LineWidth', 0.5); 
hold off;

subplot(2,2,3); plot(t, p);   title('Winkel');            ylabel('phi [rad]');     xlabel('t [s]'); ylim([-0.7 0.7]); grid on
hold on;
% Wir nehmen den Zeitstempel an dem der Index t_60 erreicht wird
zeitpunkt = t(t_60); 
xline(zeitpunkt, '--','Color', 'r', 'LineWidth', 0.5); 
hold off;
subplot(2,2,4); plot(t, p_dot);      title('Winkelgeschwindigkeit');    ylabel('w [rad/s]'); xlabel('t [s]'); ylim([-0.7 0.7]);  grid on

hold on;
% Wir nehmen den Zeitstempel an dem der Index t_60 erreicht wird
zeitpunkt = t(t_60); 
xline(zeitpunkt, '--','Color', 'r', 'LineWidth', 0.5); 
hold off;
clear
clc

%% Variablen
syms x(t) phi(t)
syms mW mP l g real

%% Hilfsvariablen
x_dot    = diff(x,t);
phi_dot  = diff(phi,t);

x_ddot   = diff(x,t,2);
phi_ddot = diff(phi,t,2);

%% Energien
T = 1/2*mW*x_dot^2 + 1/2*mP*( x_dot^2 + 2*l*x_dot*phi_dot*cos(phi) + l^2*phi_dot^2 );

V = mP*g*l*(1-cos(phi));

%% Lagrange
Lag = T - V;

%% Euler-Lagrange
EL_x = simplify( diff(diff(Lag,x_dot),t) - diff(Lag,x));

EL_phi = simplify( diff(diff(Lag,phi_dot),t) - diff(Lag,phi));

%% Anzeigen
disp('EL_x = ')
pretty(EL_x)

disp('EL_phi = ')
pretty(EL_phi)

%% Neue symbolische Variablen für solve
syms XDD PHIDD real

EL_x_sub = subs(EL_x, [x_ddot, phi_ddot], [XDD, PHIDD]);

EL_phi_sub = subs(EL_phi, [x_ddot, phi_ddot],  [XDD, PHIDD]);

%% Nach Beschleunigungen lösen
sol = solve( [EL_x_sub == 0, EL_phi_sub == 0], [XDD, PHIDD]);

%% Ergebnisse
x_ddot_sol = simplify(sol.XDD)
phi_ddot_sol = simplify(sol.PHIDD)

disp('x_ddot = ')
pretty(x_ddot_sol)

disp('phi_ddot = ')
pretty(phi_ddot_sol)
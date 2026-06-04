% Polynomial Degree

function [rq, rqd, rqdd] = Polynom5DegreeTrajectory(qi, qdi, qddi, qf, qdf, qddf, tf, t)
    % Polynom5DegreeTrajectory Berechnet die Soll-Position, -Geschwindigkeit 
    % und -Beschleunigung für ein Polynom 5. Grades zu einem Zeitpunkt t.
    %
    % Inputs:
    %   qi, qf     = Start- und Endposition
    %   qdi, qdf   = Start- und Endgeschwindigkeit
    %   qddi, qddf = Start- und Endbeschleunigung
    %   tf         = Gesamtdauer der Trajektorie (Trajectory Duration)
    %   t          = Aktueller Zeitpunkt, für den berechnet werden soll
    %
    % Outputs:
    %   rq         = Parametrisierte Soll-Position bei Zeit t
    %   rqd        = Parametrisierte Soll-Geschwindigkeit bei Zeit t
    %   rqdd       = Parametrisierte Soll-Beschleunigung bei Zeit t

    % Koeffizienten des Polynoms berechnen (wie im Bild)
    a0 = qi;
    a1 = qdi;
    a2 = 0.5 * qddi;
    
    a3 = (1 / (2 * (tf^3)))  * (-20 * (qi - qf) - (12 * qdi + 8 * qdf) * tf - (3 * qddi - qddf) * tf * tf);
    a4 = (1 / (2 * (tf^4)))  * ( 30 * (qi - qf) + (16 * qdi + 14 * qdf) * tf + (3 * qddi - 2 * qddf) * tf * tf);
    a5 = (1 / (2 * (tf^5)))  * (-12 * (qi - qf) -  6 * (qdi + qdf) * tf - (qddi - qddf) * tf * tf);

    % Soll-Werte für den spezifischen Zeitpunkt t berechnen
    rq   = a0 + a1*t + a2*t^2 +   a3*t^3 +   a4*t^4 +   a5*t^5;
    rqd  =      a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4;
    rqdd =           2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3;
end
%% Aufgabe 1 Weihnachtsprojekt: Bestimmung der Kinematik
% gegebene Werte:
l1 = 0.16;
l2 = 0.128;


% Berechnung Transformationsmatrizen:
syms theta_1 theta_2
T_1 = dhtranssym('id','1','a',0,'alp',0,'d',0,'theta', theta_1);
T_2 = dhtranssym('id','2','a',l1,'alp',0,'d',0, 'theta', theta_2);
T_3 = dhtranssym('id','3','a',l2,'alp',0,'d',0,'theta',0);
T = T_1 * T_2 * T_3;

%Theta_1 und Theta_2 mit alpha-pi/2 bzw. beta ersetzen:
syms alpha
syms beta

theta_1_soll = solve(alpha == theta_1 + pi/2,theta_1);
theta_2_soll = solve(beta == theta_2, theta_2);
T_1 = simplify(subs(T_1,theta_1,theta_1_soll));
T_2 = subs(T_2,theta_2,theta_2_soll);


%% Aufgabe 1 Weihnachtsprojekt: Bestimmung der Kinematik

% gegebene Werte:
l1 = 0.16;
l2 = 0.128;


% Berechnung Transformationsmatrizen:
syms theta_1 theta_2
T_1 = dhtranssym('id','1','a',0,'alp',0,'d',0);
T_2 = dhtranssym('id','2','a',l1,'alp',0,'d',0);
T_3 = dhtranssym('id','3','a',l2,'alp',0,'d',0,'theta',0);
T = T_1 * T_2 * T_3;

%Theta_1 und Theta_2 mit alpha-pi/2 bzw. betha ersetzen:
syms t
syms alpha(t)
syms beta(t)

theta_1_soll = solve(alpha == theta_1 + pi/2,theta_1);
theta_2_soll = solve(beta == theta_2, theta_2);
T_1 = simplify(subs(T_1,theta_1,theta_1_soll));
T_2 = subs(T_2,theta_2,theta_2_soll);




%Inserting real values:
% % % theta_1 = -pi/2;  %(alpha)
% % % theta_2 = pi/4;  %(betha)
% % % P = vpa(subs(T_1 * T_2 * [0 0 0 1]'));
% % % K = vpa(subs(T*[0 0 0 1]'));
% % % 
% % % %Plotting:
% % % plot([0 P(1)],[0 P(2)],'y','LineWidth',4)
% % % axis([-0.3 0.3 -0.3 0.05])
% % % hold on
% % % plot([P(1) K(1)],[P(2) K(2)],'g', 'LineWidth',4)
% % % hold off
%test

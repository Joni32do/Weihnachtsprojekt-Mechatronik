%% Aufgabe 1 Weihnachtsprojekt: Bestimmung der Kinematik
function [T_02, T_03] = dh_trafo()
    % gegebene Werte:
    l1 = 0.16;
    l2 = 0.128;
    
    % Berechnung Transformationsmatrizen:
    alpha = sym("alpha");
    beta = sym("beta");
    
    T_1 = dhtranssym('id','1','a',0,'alp',0,'d',0,'theta', alpha - pi/2);
    T_2 = dhtranssym('id','2','a',l1,'alp',0,'d',0, 'theta', beta);
    T_3 = dhtranssym('id','3','a',l2,'alp',0,'d',0,'theta',0);
    
    T_02 = matlabFunction(T_1 * T_2);
    T_03 = matlabFunction(T_1 * T_2 * T_3);
end

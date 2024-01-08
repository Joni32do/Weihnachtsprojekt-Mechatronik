%% Bestimmung der Bewegungsgleichungen (Aufgabe 2)
run kinematik.m

%verallgemeinerte Koordinaten:
y = [alpha; beta];
y_punkt = diff(y,t);

%% Transformationsmatrizen für Schwerpunke Ki,s:
T_1_1s = dhtranssym('id','1','a',l1/2,'alp',0,'d',0 , 'theta',0);
T_2_2s = dhtranssym('id','2','a',l2/2,'alp',0,'d',0 , 'theta',0);

%% Jakobimatrizen für Translation:

%Schwerpunkte Körper 1 und 2 im Inertialsystem:
r_s1_0 = T_1*T_1_1s * [0; 0; 0; 1];
r_s2_0 = simplify(T_1*T_2*T_2_2s * [0; 0; 0; 1]);

%Jacobimatrizen der Translation Körper 1 und 2:
J_t1 = jacobian(r_s1_0(1:3),y);
J_t2 = simplify(jacobian(r_s2_0(1:3),y));

%% Jakobimatrizen der Rotation:
S_1s = (T_1*T_1_1s);
S_1s = S_1s(1:3, 1:3);
s1 = S_1s*[0; 0; alpha];

S_2s = T_1*T_2*T_2_2s;
S_2s = S_2s(1:3, 1:3);
s2 = S_2s*[0; 0; alpha+beta];

J_r1 = jacobian(s1, y);
J_r2 = jacobian(s2, y);

%% Berechnung potentielle Energie U:
g = 9.81; %Erdbeschleunigung
m1 = 0.1817;
m2 = 0.0944;

V1 = simplify(m1*g*r_s1_0(2)); % potentielle Energie Körper 1
V2 = simplify(m2*g*r_s2_0(2)); % potentielle Energie Körper 2
V = simplify(V1 + V2);

%% Berechnung kinetischer Energie:
%Trägheitstensoren (in phi1, phi2 umbenannt) im Schwerpunkt iS:
phi1 = zeros(3);
phi1(3,3) = 0.0309;
phi2 = zeros(3);
phi2(3,3) = 0.0045;

M1 = simplify(m1*(J_t1')*J_t1 + J_r1'*S_1s*phi1*S_1s'*J_r1);
M2 = simplify(m2*(J_t2')*J_t2 + J_r2'*S_2s*phi2*S_2s'*J_r2);

M = M1 + M2;
M = simplify(M);
% Tkin = 0.5*y_punkt'*M*y_punkt;

%% Lagrange-Funktion
% L = Tkin - V;
% L = simplify(L);

%% Reibmoment:
% erste Approximation:
Mreib_1 = 3.843e-06*diff(alpha,t);
Mreib_2 = 3.887e-06*diff(beta,t);

Q = [Mreib_1; Mreib_2];

%% Gesamtgleichung:
% in der Form: M(y)*y'' + D(y,y')*y' + g(y) = Reibmoment

% Gravitationsvektor g:
g = [diff(V,y(1)); diff(V,y(2))];

% Auffüllen der Matrix D mit Christoffel-Symbolen:
D = sym('D', [2,2]);
y_punkt_ = y_punkt(t);

for k = 1:2
    for j = 1:2 
        D(k,j) = christoffel(M(t),y(t),1,k,j)* y_punkt_(1) + christoffel(M(t),y(t),2,k,j) * y_punkt_(2);
        D(k,j) = simplify(D(k,j));
    end
end
D = simplify(D);
M = simplify(M);

eq = M*diff(y_punkt,t) + D*y_punkt + g == Q;







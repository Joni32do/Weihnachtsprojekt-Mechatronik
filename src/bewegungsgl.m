function symbolic_y_ddot = bewegungsgl()

%% Bestimmung der Bewegungsgleichungen (Aufgabe 2)
l1 = 0.16;
l2 = 0.128;


% This loads T1, T2, T3 which are symbolic matrices which are altered
% by changing the value of $\alpha$ and $\beta$
% run kinematik.m

%verallgemeinerte Koordinaten:
alpha = sym("alpha");
alpha_dot = sym("alpha_dot");
beta = sym("beta");
beta_dot = sym("beta_dot");
u = sym("u", )

y = [alpha; beta];
y_punkt = [alpha_dot; beta_dot];

% % Real number
% assumeAlso(y, 'real');
% assumeAlso(y_punkt, 'real');

%% Transformationsmatrizen
T_1 = dhtranssym('id','1','a',0,'alp',0,'d',0,'theta', alpha - pi/2);
T_2 = dhtranssym('id','2','a',l1,'alp',0,'d',0, 'theta', beta);
T_3 = dhtranssym('id','3','a',l2,'alp',0,'d',0,'theta',0);

% Transformationsmatrizen für Schwerpunke Ki,s:
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
Mreib_1 = 3.843e-06* y_punkt(1);
Mreib_2 = 3.887e-06* y_punkt(2);

Q = [Mreib_1; Mreib_2];

%% Gesamtgleichung:
% in der Form: M(y)*y'' + D(y,y')*y' + g(y) = Reibmoment

% Gravitationsvektor g:
g = [diff(V,y(1)); diff(V,y(2))];

% Auffüllen der Matrix D mit Christoffel-Symbolen:
D = sym('D', [2,2]);
% y_punkt_ = y_punkt(t);

for k = 1:2
    for j = 1:2 
        D(k,j) = christoffel(M,y,1,k,j)* y_punkt(1) + christoffel(M,y,2,k,j) * y_punkt(2);
        D(k,j) = simplify(D(k,j));
    end
end



%% Assembling

symbolic_y_ddot = simplify(M\(Q + u - D*y_punkt - g));



%% Testing

% M_0 = vpa(subs(M, alpha, 1));
% M_0 = vpa(subs(M_0, beta, 0.2));
% M_0 = vpa(subs(M_0, alpha_dot, 0));
% M_0 = vpa(subs(M_0, beta_dot, 1));
% issymmetric(double(M_0));
% 
% vpa(subs(g, [alpha, beta], [0, 1]));
% 
% % symbolic_y_ddot = subs(symbolic_y_ddot, alpha, pi+0.01);
% % symbolic_y_ddot = subs(symbolic_y_ddot, beta, 0);
% % symbolic_y_ddot = subs(symbolic_y_ddot, alpha_dot, 0);
% % symbolic_y_ddot = subs(symbolic_y_ddot, beta_dot, 0);
% fsa = [pi, 0, 0, 0]
% symbolic_y_ddot = subs(symbolic_y_ddot, [alpha, beta, alpha_dot, beta_dot], fsa)
% symbolic_y_ddot = subs(symbolic_y_ddot, u, 0);
% 
% vpa(symbolic_y_ddot)
end





%% Setup
clear;
clc;

%% Bewegungsgleichung

% generates `symbolic_y_ddot`
symbolic_y_ddot = bewegungsgl();
% clearvars -except symbolic_y_ddot T_02 T_03

%% Anfangswerte
% Syntax: y_0 = [alpha; alpha_dot; beta; beta_dot]
y_0 = [pi/2; 0; 0; 0];
u = 0;

tol_rel = 1e-4;
tol_abs = 1e-7;
max_step = 3*1e3;

opts = odeset('RelTol', tol_rel, 'AbsTol', tol_abs, 'MaxStep', max_step);

%% Bewegungsgleichung numerisch integrieren

% Use solver ode45

func_y_ddot = matlabFunction(symbolic_y_ddot);
odefun = @(t, y) assemble_odefun(y, u, func_y_ddot);

[t, y] = ode45(odefun, [0, 10], y_0, opts);



%% Visualization

alphas = y(:, 1);
betas = y(:, 3);
[T_02, T_03] = dh_trafo();
orig = [0 0 0 1]';

% Animation
figure(1);
for i=1:size(t)
    G2 = T_02(alphas(i), betas(i))*orig;
    G3 = T_03(alphas(i), betas(i))*orig;
    plot_robot(G2, G3)
    pause(0.01)
    drawnow;
end




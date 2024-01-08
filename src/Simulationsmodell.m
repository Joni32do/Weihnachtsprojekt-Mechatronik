%% Simulationsmodell
% MatLab Script, zur Integration und Visualisierung
% der Simulation eines Knickarmroboters
clear;
clc;

%% Anfangswerte
% Syntax: y_0 = [alpha; alpha_dot; beta; beta_dot]
y_0 = [pi/8; 0; 0; 0];
u = [-0.1; 1];
tspan = [0, 10];

opts = odeset('RelTol', 1e-4, ...
              'AbsTol', 1e-7, ...
              'MaxStep', 3*1e3);

berechne_bewegungsgleichung_immer = true;
plot_Ergebnis = false;

%% Bewegungsgleichung
if ~exist('func_y_ddot.m', 'file') || berechne_bewegungsgleichung_immer
    symbolic_y_ddot = bewegungsgl();
    matlabFunction(symbolic_y_ddot, 'file', 'func_y_ddot.m');
end

%% Bewegungsgleichung numerisch integrieren

% Use solver ode45

odefun = @(t, y) assemble_odefun(y, u, @func_y_ddot);

[t, y] = ode45(odefun, tspan, y_0, opts);



%% Visualization
if plot_Ergebnis
    alphas = y(:, 1);
    betas = y(:, 3);
    [T_02, T_03] = dh_trafo();
    orig = [0 0 0 1]';
    time = [diff(t); 0];
    
    % Animation
    figure(1);
    for i=1:size(t)
        G2 = T_02(alphas(i), betas(i))*orig;
        G3 = T_03(alphas(i), betas(i))*orig;
        plot_robot(G2, G3)
        drawnow;
        pause(time);
    end
end

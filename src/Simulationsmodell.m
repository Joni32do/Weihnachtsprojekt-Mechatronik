%% Simulationsmodell
% MatLab Script, zur Integration und Visualisierung
% der Simulation eines Knickarmroboters
clear;
clc;

%% Anfangswerte
% Syntax: y_0 = [alpha; alpha_dot; beta; beta_dot, err_alpha, err_beta]
y_0 = [pi/8; 0; 0; 0; 0; 0];
tspan = [0, 10];

opts = odeset('RelTol', 1e-4, ...
              'AbsTol', 1e-7, ...
              'MaxStep', 3*1e3);

%% Trajektorienplanung
% Ziel in globalen Koordinaten
y_end_glob = [4*sqrt(6)-4*sqrt(2)-10; ...
             -4*sqrt(6)-4*sqrt(2)-10*sqrt(3); 0]/125;

% Aus Kinematik
pos_endeff = @(q) 4/25*[sin(q(1)) + 4/5*sin(q(1)+q(2)); ...
    - cos(q(1)) - 4/5*cos(q(1)+q(2)); 0];
to_endeff = @(q) pos_endeff(q) - y_end_glob;
q0 = [0;0];
q_end_glob = fsolve(to_endeff, q0);

%% Reglerentwurf
% regler_struct
reg.r_alpha = q_end_glob(1);
reg.r_beta = q_end_glob(2);

reg.Kp = 150;
reg.Ki = 1;
reg.Kd = 10;


% assuming x -> [alpha; alpha_dot; beta; beta_dot; err_alpha; err_beta]
reg.pid = @(x) [-reg.Kp*(x(1)-reg.r_alpha)-reg.Ki*x(5)-reg.Kd*x(2);...
                -reg.Kp*(x(3)-reg.r_beta)-reg.Ki*x(6)-reg.Kd*x(4)];

%% Boolean Options

berechne_doppelt = true;
plot_Ergebnis = true;

%% Bewegungsgleichung
if ~exist('func_y_ddot.m', 'file') || berechne_doppelt
    symbolic_y_ddot = bewegungsgl();
    matlabFunction(symbolic_y_ddot, 'file', 'func_y_ddot.m');
end

%% Bewegungsgleichung numerisch integrieren

% Use solver ode45

odefun = @(t, y) assemble_odefun(y, @func_y_ddot, reg);

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
    for i=1:1000:size(t)
        G2 = T_02(alphas(i), betas(i))*orig;
        G3 = T_03(alphas(i), betas(i))*orig;
        plot_robot(G2, G3)
        hold on
        plot(y_end_glob(1), y_end_glob(2), "o")
        hold off
        drawnow;
        % pause(time);
    end
end

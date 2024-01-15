%% Simulationsmodell
% MatLab Script, zur Integration und Visualisierung
% der Simulation eines Knickarmroboters
clear;
clc;

%% Boolean Options

berechne_doppelt = true;
plot_Ergebnis = true;

%% Bewegungsgleichung
if ~exist('func_y_ddot.m', 'file') || berechne_doppelt
    symbolic_y_ddot = bewegungsgl();
    matlabFunction(symbolic_y_ddot, 'file', 'func_y_ddot.m');
end

%% Anfangswerte
% Syntax: y_0 = [alpha; alpha_dot; beta; beta_dot, err_alpha, err_beta]
y_0 = [pi/2; 0.5; -pi/5; -0.1; 0; 0];
tspan = [0, 1];

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
q_0 = [y_0(1); y_0(3)];
q_end = fsolve(to_endeff, q_0);

%% Trajektorienplanung cubic

% t_0 = tspan(1);
% t_end = tspan(2);
% 
% A = [1,  t_0,   t_0^2,    t_0^3; ...
%      0,    1,   2*t_0,  3*t_0^2; ...
%      1,t_end, t_end^2,  t_end^3; ...
%      0,    1, 2*t_end, 3*t_end^2];
% % Vector of initial and final joint positions and velocities
% a_init = [q_0(1); y_0(2); q_end(1); 0];
% b_init = [q_0(2); y_0(4); q_end(2); 0];
% 
% a_coef = A\a_init;
% b_coef = A\b_init;
% 
% a_poly = @(t) a_coef(1) + a_coef(2)*t + a_coef(3)*t.^2 + a_coef(4)*t.^3;
% b_poly = @(t) b_coef(1) + b_coef(2)*t + b_coef(3)*t.^2 + b_coef(4)*t.^3;

%% Trajektorienplanung quintic
t_0 = tspan(1);
t_end = tspan(2);

 A = [1, 0, 0, 0, 0, 0;
         0, 1, 0, 0, 0, 0;
         0, 0, 1, 0, 0, 0;
         1, 1*(t_end-t_0), 1*(t_end-t_0).^2, 1*(t_end-t_0).^3, 1*(t_end-t_0).^4, 1*(t_end-t_0).^5;
         0, 1, 2*(t_end-t_0), 3*(t_end-t_0).^2, 4*(t_end-t_0).^3, 5*(t_end-t_0).^4;
         0, 0, 2, 6*(t_end-t_0), 12*(t_end-t_0).^2, 20*(t_end-t_0).^3];

%[t_0; q_0; v_0; a_0; t_end; q_end; v_end; a_end]
a_init = [ q_0(1); y_0(2); 0;  q_end(1); 0; 0];
b_init = [ q_0(2); y_0(4); 0;  q_end(2); 0; 0];

a_coef = A\a_init;
b_coef = A\b_init;

a_poly = @(t)  a_coef(1) + a_coef(2)*(t-t_0) + a_coef(3)*(t-t_0).^2 + a_coef(4)*(t-t_0).^3 + a_coef(5)*(t-t_0).^4 + a_coef(6)*(t-t_0).^5;
b_poly = @(t)  b_coef(1) + b_coef(2)*(t-t_0) + b_coef(3)*(t-t_0).^2 + b_coef(4)*(t-t_0).^3 + b_coef(5)*(t-t_0).^4 + b_coef(6)*(t-t_0).^5;

% ploting the trajectories
t = t_0:(t_end-t_0)/500:t_end;   
figure(2);
    subplot(1,2,1);
    plot(t,a_poly(t));
    leg=get(legend,'String');
    legend(leg);
    xlabel('time [s]'); ylabel('joint angle alpah [rad]');
        subplot(1,2,2);
    plot(t,b_poly(t));
    leg=get(legend,'String');
    legend(leg);
    xlabel('time [s]'); ylabel('joint angle beta [rad]');
    hold on;
%% Reglerentwurf
% regler_struct
reg.r_alpha = a_poly;
reg.r_beta = b_poly;

% Noise in friction is the same as noisy u
reg.noise_amp = 5e-3;

reg.Kp = 150;
reg.Ki = 1;
reg.Kd = 10;


% assuming x -> [alpha; alpha_dot; beta; beta_dot; err_alpha; err_beta]
reg.pid = @(t, x) [-reg.Kp*(x(1)-reg.r_alpha(t))-reg.Ki*x(5)-reg.Kd*x(2);...
                   -reg.Kp*(x(3)- reg.r_beta(t))-reg.Ki*x(6)-reg.Kd*x(4)];



%% Bewegungsgleichung numerisch integrieren

% Use solver ode45

odefun = @(t, y) assemble_odefun(t, y, @func_y_ddot, reg);

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
    % plots maximal 250 frames
    max_frame = 250;
    n_frame = min(size(y,1), max_frame);
    for frame=1:n_frame
        i = floor(frame/n_frame .* numel(t));
        G2 = T_02(alphas(i), betas(i))*orig;
        G3 = T_03(alphas(i), betas(i))*orig;
        plot_robot(G2, G3)
        hold on
        plot(y_end_glob(1), y_end_glob(2), "o")
        hold off
        drawnow;
        %wait such that the animation is as long as the simulated time
        pause(time(i));
    end
end

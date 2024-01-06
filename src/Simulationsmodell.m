%% Load Bewegungsgleichung

% generates `symbolic_y_ddot` 
run bewegungsgl.m
clearvars -except symbolic_y_ddot
% TODO: make this a function

%% Anfangswerte
% Syntax: y_0 = [alpha; alpha_dot; beta; beta_dot]
y_0 = [pi/2; 0; 0; 0];
u = 0;

tol_rel = 1e-4;
tol_abs = 1e-7;
max_step = 3*1e3;


%% Bewegungsgleichung numerisch integrieren

% Use solver ode45

odefun = @(t, y) assemble_odefun(y, u, symbolic_y_ddot);

[t, y] = ode45(odefun, [0, 10], y_0);




q = [y(:, 1), y(:, 3)];
% Animation
figure(1);
for i=1:size(t)
    plot_robot(q(i, :))
    drawnow;
end




function dy = assemble_odefun(t, y, func, reg)
% assuming y -> [alpha; alpha_dot; beta; beta_dot; err_alpha; err_beta]
    dy = zeros(6,1);

    u = reg.pid(t, y);
    
    % limit u values
    u_min = -1;
    u_max = 1;
    
    u = min(u_max, max(u_min, u));

    % Noise in friction can also be added to u
    noise = rel.noise_amp *2*(rand(2,1)-0.5);
    u = u + noise;

    % Use func
    y_ddot = func(y(1), y(2), y(3), y(4), u(1), u(2));

    dy(1) = y(2);
    dy(2) = double(y_ddot(1));
    dy(3) = y(4);
    dy(4) = double(y_ddot(2));
    % error from soll-value
    dy(5) = y(1) - reg.r_alpha(t);
    dy(6) = y(2) - reg.r_beta(t);
end




    % ALTERNATIVE:

    % Substitute values in symbolic form
    
    % y_ddot = subs(symbolic_y_ddot, [alpha, beta, alpha_dot, beta_dot], y);
    
    % y_ddot = subs(symbolic_y_ddot, alpha, y(1));
    % y_ddot = subs(y_ddot, alpha_dot, y(2));
    % y_ddot = subs(y_ddot, beta, y(3));
    % y_ddot = subs(y_ddot, beta_dot, y(4));

    % y_ddot = vpa(subs(y_ddot, u, u_));
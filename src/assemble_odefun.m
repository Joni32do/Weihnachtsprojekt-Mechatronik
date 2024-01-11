function dy = assemble_odefun(y, func, reg)
    dy = zeros(6,1);
    % error from soll-value
    dy(5) = y(1) - reg.r_alpha;
    dy(6) = y(2) - reg.r_beta;

    u = reg.pid(y);
    y_ddot = func(y(1), y(2), y(3), y(4), u(1), u(2));

    dy(1) = y(2);
    dy(2) = double(y_ddot(1));
    dy(3) = y(4);
    dy(4) = double(y_ddot(2));
end




    % ALTERNATIVE:

    % Substitute values in symbolic form
    
    % y_ddot = subs(symbolic_y_ddot, [alpha, beta, alpha_dot, beta_dot], y);
    
    % y_ddot = subs(symbolic_y_ddot, alpha, y(1));
    % y_ddot = subs(y_ddot, alpha_dot, y(2));
    % y_ddot = subs(y_ddot, beta, y(3));
    % y_ddot = subs(y_ddot, beta_dot, y(4));

    % y_ddot = vpa(subs(y_ddot, u, u_));
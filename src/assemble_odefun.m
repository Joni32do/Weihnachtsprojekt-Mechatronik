function dy = assemble_odefun(y, u_, func)
    % Only works if `symbolic_y_ddot` is loaded in workspace`
    dy = zeros(4,1);
    y_ddot = func(y(1), y(2), y(3), y(4), u_);

    % ALTERNATIVE:

    % Substitute values in symbolic form
    % y_ddot = subs(symbolic_y_ddot, [alpha, beta, alpha_dot, beta_dot], y);
    % y_ddot = subs(symbolic_y_ddot, alpha, y(1));
    % y_ddot = subs(y_ddot, alpha_dot, y(2));
    % y_ddot = subs(y_ddot, beta, y(3));
    % y_ddot = subs(y_ddot, beta_dot, y(4));
    % y_ddot = vpa(subs(y_ddot, u, u_));
    
    dy(1) = y(2);
    dy(2) = double(y_ddot(1));
    dy(3) = y(4);
    dy(4) = double(y_ddot(2));
    

end
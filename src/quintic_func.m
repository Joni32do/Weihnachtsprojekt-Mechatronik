function quintic_spline_function = quintic_func(t0, q0, v0, a0, tf, qf, vf, af)
    % Input:
    % t0, tf: Initial and final time
    % q0, v0, a0: Initial position, velocity, and acceleration
    % qf, vf, af: Final position, velocity, and acceleration

    %
    % q_t(t) = a_i + b_i*(t-t0) + c_i*(t-t0)^2 + d_i*(t-t0)^3 + e_i*(t-t0)^4 + f_i*(t-t0)^5;
    % q_t_dot(t) = b_i + 2*c_i*(t-t0) + 3*d_i*(t-t0)^2 + 4*e_i*(t-t0)^3 + 5*f_i*(t-t0)^4;
    % q_t_ddot(t) = 2*c_i + 6*d_i*(t-t0) + 12*e_i*(t-t0)^2 + 20*f_i*(t-t0)^3;

    
    % Define matrix A
    A = [1, 0, 0, 0, 0, 0;
         0, 1, 0, 0, 0, 0;
         0, 0, 1, 0, 0, 0;
         1, 1*(tf-t0), 1*(tf-t0)^2, 1*(tf-t0)^3, 1*(tf-t0)^4, 1*(tf-t0)^5;
         0, 1, 2*(tf-t0), 3*(tf-t0)^2, 4*(tf-t0)^3, 5*(tf-t0)^4;
         0, 0, 2, 6*(tf-t0), 12*(tf-t0)^2, 20*(tf-t0)^3];

    % Define vector B
    B = [q0; v0; a0; qf; vf; af];

    % Find one possible solution using the pseudo-inverse
    x = A \ B;

   

    t = t0:(tf-t0)/500:tf;   
    q = x(1) + x(2)*(t-t0) + x(3)*(t-t0).^2 + x(4)*(t-t0).^3 + x(5)*(t-t0).^4 + x(6)*(t-t0).^5;
    qdot = x(2) + 2*x(3)*(t-t0) + 3*x(4)*(t-t0).^2 + 4*x(5)*(t-t0).^3 + 5*x(6)*(t-t0).^4;
    qddot = 2*x(3) + 6*x(4)*(t-t0) + 12*x(5)*(t-t0).^2 + 20*x(6)*(t-t0).^3;



    % Plot the trajectories
    figure(1);
    subplot(1,3,1);
    plot(t,q,'r-.','LineWidth',2);
    leg=get(legend,'String');
    legend(leg);
    xlabel('time [s]'); ylabel('joint angle [rad]');
    hold on;
    subplot(1,3,2);
    plot(t,qdot,'r-.','LineWidth',2);
    leg=get(legend,'String');
    legend(leg);
    xlabel('time [s]'); ylabel('veclocity [rad/s]');
    hold on;
    subplot(1,3,3);
    plot(t,qddot,'r-.','LineWidth',2);
    leg=get(legend,'String');
    legend(leg);
    xlabel('time [s]'); ylabel('acceleration [rad/s^2]');
    hold on;

    %return the quintic function
     syms t;
     quintic_spline  =  [x(1);x(2)*(t-t0);x(3)*(t-t0)^2;x(4)*(t-t0)^3;x(5)*(t-t0)^4;x(6)*(t-t0)^5];
     quintic_spline_function = matlabFunction(quintic_spline, 'Vars', t);
     disp(quintic_spline_function);

end

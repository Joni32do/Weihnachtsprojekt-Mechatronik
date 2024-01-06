function plot_robot(q)
    l1 = 0.16;
    l2 = 0.128;

    syms theta_1 theta_2
    origin = [0 0 0 1]';
    T_1 = dhtranssym('id','1','a',0,'alp',0,'d',0,'theta', theta_1-(pi/2));
    T_2 = dhtranssym('id','2','a',l1,'alp',0,'d',0, 'theta', theta_2);
    T_3 = dhtranssym('id','3','a',l2,'alp',0,'d',0,'theta',0);
    T = T_1 * T_2 * T_3;
    
        
    theta_1 = q(1);
    theta_2 = q(2);

    %Inserting real values:
    G2 = vpa(subs(T_1 * T_2 * origin));
    G3 = vpa(subs(T*origin));

    %Plotting:
    plot([0 G2(1)],[0 G2(2)],'y','LineWidth',4)
    axis([-0.3 0.3 -0.3 0.05])
    hold on
    plot([G2(1) G3(1)],[G2(2) G3(2)],'g', 'LineWidth',4)
    hold off


end
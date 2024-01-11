function plot_robot(G2, G3, full)
    if nargin > 2
            alpha = G2;
            beta = G3;
            [T_02, T_03] = dh_trafo();
            orig = [0 0 0 1]';
            G2 = T_02(alpha, beta)*orig;
            G3 = T_03(alpha, beta)*orig;
    end
    %Plotting:
    plot([0 G2(1)],[0 G2(2)],'y','LineWidth',4)
    axis([-0.3 0.3 -0.3 0.05])
    hold on
    plot([G2(1) G3(1)],[G2(2) G3(2)],'g', 'LineWidth',4)
    hold off
end
function plot_robot(G2, G3)
    %Plotting:
    plot([0 G2(1)],[0 G2(2)],'y','LineWidth',4)
    axis([-0.3 0.3 -0.3 0.05])
    hold on
    plot([G2(1) G3(1)],[G2(2) G3(2)],'g', 'LineWidth',4)
    hold off
end
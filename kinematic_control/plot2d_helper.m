function plot2d_helper(x)
    % x shape: [2, horizon]
    plot(x(1, :), x(2, :))
    hold on
    grid on
    axis equal
    title('Object trajectory')
    xlabel('x_1')
    ylabel('x_2')
%     zlabel('t')
end
function plot_force_helper(tspan, force_robots_buf)
% This function plots follower and leader forces
    % force buffer shape: [N+1, 2, horizon]
    % with leader force at (1, :, :) and follower forces at (2:end, :, :)
    global N
    for dim = 1 : 2
        subplot(2, 1, dim)
        % leader robot
        force_leader = squeeze(force_robots_buf(1, dim, :));
        plot(tspan, force_leader)
        grid on
        % follower robot(s)
        hold on
        for i = 2 : N + 1
            force_follower_i = squeeze(force_robots_buf(i, dim, :));
            plot(tspan, force_follower_i, '-.')
            fprintf("%s", int2str(i))
        end
        legend("robot " + string(1 : N+1))
        xlabel('t')
        ylabel('F' + string(dim))
        hold on
    end
    subplot(2, 1, 1)
    title('Robot forces')
    hold off
end


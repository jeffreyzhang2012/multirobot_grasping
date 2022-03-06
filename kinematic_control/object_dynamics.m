function [dvdt, F_robots, F_env] = object_dynamics(v, w, vd, wd)
% This function simulates the object velocity update step.
    global mu_s, global M, global N, global g, global EPS
    F_robot = zeros(2, 1);      % sum of robot forces
    F_robots = zeros(N + 1, 2); % buffer that stores all forces
    %% Environmental forces
    F_env = -mu_s * M * g .* v / (norm(v) + EPS);
    %% Leader force
    F_leader = controller_leader(v, w, vd, wd);
    F_robot = F_robot + F_leader;
    % log force into buffer
    F_robots(1, :) = F_leader;
%     fprintf("Force from leader shape: (%i, %i)\n",...
%         size(F_leader, 1), size(F_leader, 2))
%     fprintf("Force from leader %i: [%.2f, %.2f]\n",...
%         i, F_leader(1), F_leader(2));
    %% Follower forces
    for i = 1 : N
        F_follower = controller_follower(v);
        F_robot = F_robot + F_follower;
%         fprintf("Force from follower %i shape: (%i, %i)\n",...
%             i, size(F_follower, 1), size(F_follower, 2))
%         fprintf("Force from follower %i: [%.2f, %.2f]\n",...
%             i, F_follower(1), F_follower(2));
        % log force into buffer
        F_robots(i + 1, :) = F_follower;
    end
    %% Object dynamics
    if norm(F_robot) <= mu_s * M * g
        F = zeros(2, 1);
    elseif v == 0
        F = F_robot ./ norm(F_robot)...
                .* (norm(F_robot) - mu_s * M * g);
    else
        F = F_env + F_robot;
    end
    dvdt = F ./ M;
%     fprintf("dv/dt shape: (%i, %i)\n",...
%         size(dvdt, 1), size(dvdt, 2))
%     fprintf("F_robot: [%.2f, %.2f]\n",...
%         F_robot(1), F_robot(2));
%     fprintf("F_env: [%.2f, %.2f]\n",...
%         F_env(1), F_env(2));

end
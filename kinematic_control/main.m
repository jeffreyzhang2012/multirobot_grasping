clear all
clc
%% Global configuration
global_config();
%% Initialization
[x0, v0] = initial_spawn();
%% Control specifications
vd = [10; 1];   % desired velocity
dt = .01;       % discrete time step
tspan = 0 : dt : 5;     % time horizon
horizon = length(tspan);   % step horizon
fprintf("horizon = %i\n", horizon);
%% Object dynamics
% object velocity and position buffers
vel_buf = zeros(2, horizon);
pos_buf = zeros(2, horizon);
vel_buf(:, 1) = v0;
pos_buf(:, 1) = x0;
global N
% forces of N followers and 1 leader
force_robots_buf = zeros(N + 1, 2, horizon);
% environmental force
force_env_buf = zeros(2, horizon);
for t = 1 : horizon - 1
    [dvdt, F_robots, F_env] = object_dynamics(vel_buf(:, t), vd);
    force_robots_buf(:, :, t+1) = F_robots;
    force_env_buf(:, t+1) = F_env;
    vel_buf(:, t+1) = vel_buf(:, t) + dt .* dvdt;
    pos_buf(:, t+1) = pos_buf(:, t) + dt .* vel_buf(:, t);
end
%% Plots
plot2d_helper(pos_buf);
figure()
plot1d_helper(tspan, vel_buf, 'Object velocity', 'v');
figure()
plot_force_helper(tspan(2:end), force_robots_buf(:, :, 2:end))


function global_config()
% This function sets up configurations used in simulation.
    %% Object and environmental settings
    global mu_v, global mu_s, global M, global g, global EPS, global ref
    mu_v = .3;  % viscous friction coefficient
    mu_s = .5;  % static friction coefficient
    M = 1;      % object mass, kg
    g = 9.8;    % gravitational acceleration, m/s^2
    EPS = 1e-9; % small value for numerical stability
    ref = 'global'; % whether the controllers use local/global ref frame
    %% Followers
    global N
    N = 3;      % number of follower robots
    %% Leader
    global Kf, global Kt
    Kf = 1;    % proportional controller coefficient for translation
    Kt = .5;    % proportional controller coefficient for rotation
    fprintf("Set global variables done\n")
end
close all; clear all; clc;
%% Configuration:
%   F_max = 3N
%   mu_1 = 0.1
%% Object property
M = 2;
J = .3;
mu0 = 0;%0.2;
mu1 = 0.1;
g = 9.8;
initial_vel = [0.0001 0 0];
initial_pos = [0 0 0];
initial_acc = [0 0];
% TODO: add a max veclocity for object
%% Simulation and plot
m = model(M,J, mu0, mu1, g, initial_vel, initial_pos, initial_acc);
dt = 0.1;
for t = 0 : dt : 10
    m = m.dynamic_update(dt, t);
    m.draw(t);
end
figure(1)
axis([-15,50,-15,50]);
legend
title("Robot Trajectory");
%% helper functions


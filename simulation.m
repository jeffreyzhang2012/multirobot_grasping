close all; clear all; clc;
% Object property
M = 10;
J = 5;
mu0 = 0.2;
mu1 = 0.01;
g = 9.8;
initial_vel = [0 0 0];
initial_pos = [0 0 0];
% TODO: add a max veclocity for object
m = model(M,J, mu0, mu1, g, initial_vel, initial_pos);
global running;
fig = figure;
play = tic();
set(gcf,'KeyPressFcn',@keyboardEventListener);
title('Press q to close');
running = 1;
dt = 0.05;
while running && toc(play) < 10
    t_loopstart = tic();
%     m = m.update(toc(play));
    m = m.dynamic_update(dt,toc(play));
    m.draw();
    axis([-15,50,-15,50]);
    el_time = toc(t_loopstart);
    pause(dt-el_time);
    key = pollKeyboard();
    if(key ~= false); processKey(key); end
end
m.draw();
legend
title("Robot Trajectory");
%% helper functions
function processKey(key)
    global running
    if(strcmp(key,'q'))
        running = 0;
        close all;
    end
end

function keyboardEventListener(~,event)
    %keyboardEventListener Invoked when a keyboard character is pressed.
    global keypressFrame;
    global keypressDataReady;
    global keypressKey;
    keypressFrame = keypressFrame + 1;
    keypressDataReady = 1;
    keypressKey = event.Key;
end

function res = pollKeyboard()
    %pollKeyboard Waits until the callback says there is new data.
    % This routine is useful when you want to be able to capture
    % and respond to a keypress as soon as it arrives.
    % To use this, execute the following line:
    % kh = event.listener(gcf,'KeyPressFcn',@keyboardEventListener);
    % before calling this function.
    global keypressDataReady;
    global keypressKey;
    keyboardDataReadyLast = keypressDataReady;
    keypressDataReady = 0;
    if(keyboardDataReadyLast)
        res = keypressKey;
        else
        res = false;
    end
end
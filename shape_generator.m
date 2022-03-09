clear all; close all; clc;

figure;
axis([-10,10,-10,10]);
n_side = 5;
n_robot = 4;
title("Input Object Shape");
[objectX,objectY] = ginput(5);
plot(polyshape(objectX,objectY));
robot_locations = zeros(2,n_robot);
robot_attach = zeros(2,n_robot);
COM_to_robots = zeros(2,n_robot);
object = [objectX';objectY'];
title("Input Erroneous COM");
COM_with_error = ginput(1)';
for i = 1:n_robot
    title("Input Robot Position");
    [robotX,robotY] = ginput(1);
    title("Input Attachment Position");
    [attachX,attachY] = ginput(1);
    robot_locations(:,i) = [robotX,robotY];
    robot_attach(:,i) = [attachX,attachY];
    line([robotX, attachX],[robotY,attachY]);
    COM_to_robots(:,i) = [attachX - COM_with_error(1), attachY - COM_with_error(2)];
end
title("DONE");
save('configuration.mat','robot_locations','robot_attach','object','n_side','n_robot','COM_with_error','COM_to_robots')


% Compute Trajectory

Vdes = 0.6;          % Velocity of transport
dt = 0.01;           % Time step
Trajectory = 2;

% Trajectory 1: Straight Path
% Trajectory 2: U Turn
% Trajectory 3: S shape

% Output:
% Trajectory Array [TimeStep x(t) y(t) v(t) w(t)]

if Trajectory == 1
    Start = [0 0];                  End = [1 1];
    PathLength = norm(End-Start);   dX = Vdes*dt;
    theta = atan2d(End(2)-Start(2),End(1)-Start(1));
    tau = PathLength/Vdes;
    TrajArray = zeros(ceil(tau/dt),5);
    TrajArray(:,1) = 0:dt:tau;
    TrajArray(:,2) = Start(1):dX*cosd(theta):End(1);
    TrajArray(:,3) = Start(2):dX*sind(theta):End(2);
    TrajArray(:,4) = Vdes*ones(size(TrajArray,1),1);
elseif Trajectory == 2
    Start = [0 1];    Seg1 = [1 1];    Seg2 = [1 -1];    End = [0 -1];
    R = abs(Seg1(2) - Seg2(2))/2;
    Line1 = norm(Seg1-Start);    Line2 = R*pi;    Line3 = norm(End-Seg2);
    PathLength = Line1 + Line2 + Line3;    dX = Vdes*dt;
    tau = PathLength/Vdes;
    TrajArray = zeros(ceil(tau/dt),5);
    TrajArray(:,1) = 0:dt:tau;
    h = Start(1):dX:Seg1(1);
    TrajArray(1:length(h),2) = h;
    TrajArray(1:length(h),3) = Start(2)*ones(length(h),1);
    TrajArray(ceil(tau/dt)-length(h)+1:end,2) = flip(h);
    TrajArray(ceil(tau/dt)-length(h)+1:end,3) = End(2)*ones(length(h),1);
    circle = Line2/dX;
    dtheta = pi/circle;
    theta = pi/2;
    for i = 1:circle
        TrajArray(length(h)+i,2) = Seg1(1)+R*cos(theta);
        TrajArray(length(h)+i,3) = Seg1(2)-R+R*sin(theta);
        TrajArray(length(h)+i,5) = -dtheta/dt;
        theta = theta - dtheta;
    end
    TrajArray(:,4) = Vdes*ones(size(TrajArray,1),1);
elseif Trajectory == 3
    % Work InProgress
end

% Visualization

% XY = [1; 0];
% for i=1:size(TrajArray,1)
%     Rot = [cos(dtheta) -sin(dtheta); sin(dtheta) cos(dtheta)];
%     XY = Rot*XY;
%     q = quiver(TrajArray(i,2),TrajArray(i,3),Vdes*XY(1),Vdes*XY(2),'r');
%     q.LineWidth = 1.25;
%     q.MaxHeadSize = 0.5;
%     grid on
%     dtheta = TrajArray(i,5)*dt;
%     axis([-.5 2.5 -1.5 1.5])
%     pause(dt)
% end
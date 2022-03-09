classdef habibi_controller
    properties
        n_robot;
        InitConf;
        Heading;
        d; % infrared sensor range
        MaxNoise;
        Tree;
        Graph;
        RelPos;
        robotVelocities;
        robotMass;
    end

    methods
        %% initialize
        function obj = habibi_controller(n_robot, robot_locations, robotMass, infraredRange)
            obj.n_robot = n_robot;
            obj.InitConf = [1:n_robot; robot_locations].';%TODO
            obj.Heading = zeros(size(obj.InitConf,1),1); % should we initialize to 0?
            % Infrared sensor range
            obj.d = infraredRange;
            % Maximum noise of the infrared sensors
            obj.MaxNoise = 0.5;
            
            % Finding Initial Relative Positions and Orientations among agents.
            
            [obj.Tree, obj.Graph] = CreateTree(obj.InitConf,obj.d);
            obj.RelPos = RelativePosition(obj.InitConf, obj.Graph, n_robot);
            obj.robotVelocities = zeros(n_robot, 2); % initial robot speed to be [0, 0]
            obj.robotMass = robotMass;
        end

        function [obj, F] = getControlOutput(obj, GuidePos, Vdes, omega, dt)
%             GuidePos = [5,0];   % Varies with time
%             Vdes = 1;           % Varies with time
%             omega = 0.01;       % Varies with time
            
            %[GuidePos,Vdes,omega] =%OBJECTFUNCTION%
            C = CentroidEstimation(obj.RelPos,obj.Tree,obj.MaxNoise);
            DirTrans = FindDirTrans(obj.InitConf,obj.Heading,C,GuidePos,obj.MaxNoise);
            V = FindVelocity(DirTrans,omega,Vdes,C);
            [RelPos,Heading,Heading_past,V] = Update(RelPos,Heading,V,15);
            
            %%%%%% Robot frame to object frame
            for i = 1:size(V,1)
                v_local = [V(i,1:2)'; 1];
                v_object = rotz(-Heading_past(i))*v_local; 
                V(i,:) = v_object(1:2);
            end
            %%%%%%%%%%
            
            obj.robotVelocities
            F = getAppliedForce(V(:,1:2), obj.robotVelocities, obj.robotMass, dt); % TODO: noise free velocity?
            
            %%%%%% Object frame to global frame
            for i = 1:size(F,1)
                f_object = [F(i,:)'; 1];
                f_global = rotz(-rad2deg(pose(3)))*f_object; 
                F(i,:) = f_global(1:2);
            end
            %%%%%%%%%%
            
            %TODO: need to transfer F back to global frame
            obj.robotVelocities = V(:,1:2);
        end
    end
end

function F = getAppliedForce(V, Vprevious, robotMass, dt)
    accel = (V - Vprevious)./dt;
    F = accel .* robotMass;
end

function RelPos = RelativePosition(InitConf, Graph, n_robots)

% Input
% InitConf: Matrix containing in the first column the robot identifier
% and in the second and third column respectevely the x and y position of 
% each robot for the initial configuration of the system in any frame.
% Graph: Graph: Graph representing the communication among robots.
%
% Output:
% RelPos: Cell Array for each robot containing the position vector Ruv from
% robot i to robot j (j belongs to the neighbor set of robot i), and the
% orientation Buv of Ruv in the local frame of robot i.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CHANGE HERE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Number of empty cells equal to the number of robots

% RelPos = {[], [], [], [], []};%, [], [], [], []};
RelPos = cell(1, n_robots);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DirectComm = Graph.Edges{:,:};

    for i = 1:size(DirectComm,1)
        
        j = DirectComm(i,1);    k = DirectComm(i,2);
        x = InitConf(k,2)-InitConf(j,2);
        y = InitConf(k,3)-InitConf(j,3);
        theta_1 = atan2d(y,x);
        theta_2 = atan2d(-y,-x);
        
        if theta_1 < 0
            theta_1 = 360 + theta_1;
        end
        if theta_2 < 0
            theta_2 = 360 + theta_2;
        end   
        
        RelPos{j} = [RelPos{j}; k x y theta_1];
        RelPos{k} = [RelPos{k}; j -x -y theta_2];
        
    end

end

%%

function [RelPos,Heading,Heading_past,V] = Update(RelPos,Heading,V,MaxDeltaH)

% Input:
% RelPos: Cell Array for each robot containing the position vector Ruv from
% robot i to robot j (j belongs to the neighbor set of robot i), and the
% orientation Buv of Ruv in the local frame of robot i.
% Heading: Vector that contains the ground truth of the heading of each
% robot relative to the object frame.
% V: Desired direction of transport for each robot in its local frame.
% MaxDeltaH: Maximum rotation that a robot can perform in a single time
% step.
%
% Output:
% RelPos: Adjusted relative position for each robot given an update in its
% orientation that is limited by MaxDeltaH.
% V: Velocity command in robot i local frame. If the robot has to rotate
% more than MaxDeltaH in order to output the desiered velocity, the robot
% will rotate MaxDeltaH and will not command any velocity.

    n = length(RelPos);
    Heading_past = Heading;
    
    for i = 1:n
        
        theta = atan2d(V(i,2),V(i,1));
        theta(isnan(theta))=0;
        
        if abs(theta) > MaxDeltaH
            theta = MaxDeltaH*sign(theta);
            V(i,:) = [0 0 0 0];
        end
        
        Heading(i) = Heading(i) + theta;
        
        for j = 1:size(RelPos{1,i},1)
            
            RelPos{1,i}(j,4) = RelPos{1,i}(j,4) - theta;
            RelPosNew = rotz(-theta)*[RelPos{1,i}(j,2);RelPos{1,i}(j,3);1];
            RelPos{1,i}(j,2) = RelPosNew(1);
            RelPos{1,i}(j,3) = RelPosNew(2);
            
            if RelPos{1,i}(j,4)>=360
                RelPos{1,i}(j,4) = RelPos{1,i}(j,4) - 360;
            elseif RelPos{1,i}(j,4) < 0
                RelPos{1,i}(j,4) = 360 + RelPos{1,i}(j,4);
            end
            
        end
        
    end
    
    % Noisy Velocity Command
    V = V(:,3:4);

end

%%

function V = FindVelocity(DirTrans,omega,Vdes,Centroid)

% Input:
% DirTrans: Direction of transpor for each robot in robot frame.
% omega: Desired rotational velocity.
% Vdes: Desired linear velocity.
% Centroid: Centroid estimation for each robot in its local frame
% 
% Output:
% V: Desired direction of transport for each robot in its local frame.

n = size(DirTrans,1);
V = zeros(n,4);
Omega = [0 0 omega];

for i = 1:n
    
    % Clean Velocity Command
    du = [-Centroid(i,1:2) 1];
    Vw = cross(Omega,du);
    u = Vw/norm(Vw);
    u(isnan(u)) = 0;
    beta = acos(dot(u(1:2),DirTrans(i,1:2)));
    Vx = norm(Vw)*cos(beta) + Vdes;
    if Vw(2)<0
        Vy = -norm(Vw)*sin(beta);
    else
        Vy = norm(Vw)*sin(beta);
    end
    V(i,1:2) = [Vx Vy];
    
    % Noisy Velocity Command
    du = [-Centroid(i,3:4) 1];
    Vw = cross(Omega,du);
    u = Vw/norm(Vw);
    u(isnan(u)) = 0;
    beta = acos(dot(u(1:2),DirTrans(i,3:4)));
    Vx = norm(Vw)*cos(beta) + Vdes;
    if Vw(2)<0
        Vy = -norm(Vw)*sin(beta);
    else
        Vy = norm(Vw)*sin(beta);
    end
    V(i,3:4) = [Vx Vy];

end

end


%%

function DirTrans = FindDirTrans(InitConf,Heading,Centroid,GuidePos,MaxNoise)

% Input:
% InitConf: Matrix containing in the first column the robot identifier
% and in the second and third column respectevely the x and y position of 
% each robot for the initial configuration of the system in any frame.
% Heading: Vector that contains the ground truth of the heading of each
% robot relative to the object frame.
% Centroid: Centroid estimation for each robot in its local frame
% GuidePos: Position of the leader with respect to the object's centroid
% in the frame of the object.
% MaxNoise: Maximum noise of the infrared sensors
% 
% Output:
% DirTrans: Direction of transpor for each robot in robot frame.

n = size(InitConf,1);
DirTrans = zeros(n,4);

    for i = 1:n
        
        % Clean Direction of Transport
        x = GuidePos(1) - InitConf(i,2);
        y = GuidePos(2) - InitConf(i,3);
        G = rotz(Heading(i))*[x; y; 1];
        dir = [G(1)-Centroid(i,1) G(2)-Centroid(i,2)];
        DirTrans(i,1:2) = dir/norm(dir);
        
        %Noisy Direction of Transport        
        x = x + MaxNoise*(rand-0.5);
        y = y + MaxNoise*(rand-0.5);
        G = rotz(Heading(i))*[x; y; 1];
        dir = [G(1)-Centroid(i,3) G(2)-Centroid(i,4)];
        DirTrans(i,3:4) = dir/norm(dir);

    end

end

%%

function Centroid = CentroidEstimation(RelPos,Tree,MaxNoise)

% Input:
% RelPos: Cell Array for each robot containing the position vector Ruv from
% robot i to robot j (j belongs to the neighbor set of robot i), and the
% orientation Buv of Ruv in the local frame of robot i.
% Tree: Configuration tree for the group of robots.
% MaxNoise: Maximum noise of the infrared sensors
%
% Output:
% Centroid: Centroid estimation for each robot in its local frame

    
n = length(RelPos);
Centroid = zeros(n,4);

for i = 1:n
    
    S0 = [0 0;0 0;1 1];
    Siu = S0;
    Nc = Tree((Tree(:,1) == i),:);
    
    for k = 1:n-1
        Seq = Nc(k,:);
        Seq( :, ~any(Seq,1) ) = [];
        S = FindSiu(RelPos,Seq,MaxNoise);
        Siu = Siu + S;
    end
    
    % Clean Centroid
    Centroid(i,1:2) = [Siu(1,1)/Siu(3,1) Siu(2,1)/Siu(3,1)];
    
    % Noisy Centroid
    Centroid(i,3:4) = [Siu(1,2)/Siu(3,2) Siu(2,2)/Siu(3,2)];

end

end

%%

function Siu = FindSiu(RelPos,Seq,MaxNoise)

% Input:
% RelPos: Cell Array for each robot containing the position vector Ruv from
% robot i to robot j (j belongs to the neighbor set of robot i), and the
% orientation Buv of Ruv in the local frame of robot i.
% Seq: Subtree sequence from root located at robot i to robot j (that may
% or may not belong to robot´s i neighbor set).
% MaxNoise: Maximum noise of the infrared sensors
%
% Output:
% Siu: Message that robot i (root) receives from the subtree sequence
% specifying the position of robot j in robot´s i local frame.
 
Siu = [0 0;0 0;1 1];

for i = 1:length(Seq)-1
    Bvu = RelPos{1,Seq(end-i)}...
        (RelPos{1,Seq(end-i)}(:,1) == Seq(end-i+1),4);
    Ovu = RelPos{1,Seq(end-i+1)}...
        (RelPos{1,Seq(end-i+1)}(:,1) == Seq(end-i),4);
    theta = 180 - Ovu + Bvu;
    
    % Clean Message
    M = rotz(theta);
    M(1,3) = RelPos{1,Seq(end-i)}...
        (RelPos{1,Seq(end-i)}(:,1) == Seq(end-i+1),2);
    M(2,3) = RelPos{1,Seq(end-i)}...
        (RelPos{1,Seq(end-i)}(:,1) == Seq(end-i+1),3);
    Siu(:,1) = M*Siu(:,1);
    
    % Noisy Message
    M(1,3) = M(1,3) + MaxNoise*(rand-0.5);
    M(2,3) = M(2,3) + MaxNoise*(rand-0.5);
    Siu(:,2) = M*Siu(:,2);
end

end

%%

function [Tree, Graph] = CreateTree(InitConf,d)

% Input
%
% InitConf: Matrix containing in the first column the robot identifier
% and in the second and third column respectevely the x and y position of 
% each robot for the initial configuration of the system in any frame.
% d: Inter-robot communication range.
%
% Output
% Tree: Configuration tree for the group of robots.
% Graph: Graph representing the communication among robots.

n = size(InitConf,1);
G = graph();

for i = 1:n
    for j = 1:n
        if i ~= j
            R = norm(InitConf(i,2:3)-InitConf(j,2:3));
            if R <= d
                G = addedge(G,i,j);            
            end
        end
    end
end

G = simplify(G);

Tree = [];

for i = 1:n
    for j = 1:n
        if i ~= j
            P = shortestpath(G,i,j);
            Tree = [Tree; P zeros(1,n-length(P))];
        end
    end
end

Tree( :, ~any(Tree,1) ) = [];
Graph = G;

end

%%

% Miscellaneous






        

clear all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Run this part ONLY once at the beginning

% Initial Configuration of the robots in a frame that is aligned with the
% initial direction of transport. All robots are heading in said direction.

InitConf = [1 4 0;2 2 3;3 2 -3;4 -2 2;5 -2 -2];

% Go to Line ?? and adjust the number of empty cells accordingly to the 
% number of robots.

% Initialize ground truth of the heading of each robot relative to the 
% object frame.

Heading = zeros(size(InitConf,1),1);

% Infrared sensor range

d = 5;

% Maximum noise of the infrared sensors

MaxNoise = 0.1;

% Finding Initial Relative Positions and Orientations among agents.

[Tree, Graph] = CreateTree(InitConf,d);
RelPos = RelativePosition(InitConf,Graph);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Input
% GuidePos: Leader Position Relative to Object Frame at time t.
% Vdes: Desired linear velocity of the centroid of the object towards the
% leader at time t.
% omega: Desired angular velocity of the object at time t.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MAIN CODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% while true
    
    GuidePos = [5,0];
    Vdes = 1;
    omega = 0.01;
    
    %[GuidePos,Vdes,omega] =%OBJECTFUNCTION%
    C = CentroidEstimation(RelPos,Tree,MaxNoise);
    DirTrans = FindDirTrans(InitConf,Heading,C,GuidePos,MaxNoise);
    V = FindVelocity(DirTrans,omega,Vdes,C);
    [RelPos,Heading,V] = Update(RelPos,Heading,V,15);

% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%

function RelPos = RelativePosition(InitConf, Graph)

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

RelPos = {[], [], [], [], []};%, [], [], [], []};

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

function [RelPos,Heading,V] = Update(RelPos,Heading,V,MaxDeltaH)

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
    
    for i = 1:n
        
        theta = atan2d(V(i,2),V(i,1));
        theta(isnan(theta))=0;
        
        if abs(theta) > MaxDeltaH
            theta = MaxDeltaH*sign(theta);
            V(i,:) = [0 0];
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
V = zeros(n,2);
Omega = [0 0 omega];

for i = 1:n
    
    du = [-Centroid(i,:) 1];
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
    
    V(i,:) = [Vx Vy];    

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
DirTrans = zeros(n,3);

    for i = 1:n

        x = GuidePos(1) - InitConf(i,2) + MaxNoise*(rand-0.5);
        y = GuidePos(2) - InitConf(i,3) + MaxNoise*(rand-0.5);
        G = rotz(Heading(i))*[x; y; 1];
        dir = [G(1)-Centroid(i,1) G(2)-Centroid(i,2)];
        DirTrans(i,1:2) = dir/norm(dir);
        DirTrans(i,3) = atan2d(DirTrans(i,2),DirTrans(i,1));

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
Centroid = zeros(n,2);

for i = 1:n
    
    S0 = [0;0;1];
    Siu = S0;
    Nc = Tree((Tree(:,1) == i),:);
    
    for k = 1:n-1
        Seq = Nc(k,:);
        Seq( :, ~any(Seq,1) ) = [];
        S = FindSiu(RelPos,Seq,MaxNoise);
        Siu = Siu + S;
    end
    
    Centroid(i,:) = [Siu(1)/Siu(3) Siu(2)/Siu(3)] ;

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
 
Siu = [0;0;1];

for i = 1:length(Seq)-1
    Bvu = RelPos{1,Seq(end-i)}...
        (RelPos{1,Seq(end-i)}(:,1) == Seq(end-i+1),4);
    Ovu = RelPos{1,Seq(end-i+1)}...
        (RelPos{1,Seq(end-i+1)}(:,1) == Seq(end-i),4);
    theta = 180 - Ovu + Bvu;
    M = rotz(theta);
    M(1,3) = RelPos{1,Seq(end-i)}...
        (RelPos{1,Seq(end-i)}(:,1) == Seq(end-i+1),2)...
        + MaxNoise*(rand-0.5);
    M(2,3) = RelPos{1,Seq(end-i)}...
        (RelPos{1,Seq(end-i)}(:,1) == Seq(end-i+1),3)...
        + MaxNoise*(rand-0.5);
    Siu = M*Siu;
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
% for i = 1:size(InitConf,1)
%     c = rotz(Heading(i))*[C(i,1); C(i,2); 1];
%     C(i,:) = [c(1) c(2)];
% end
%     C = C + InitConf(:,2:3);
%     plot(C(:,1),C(:,2),'x')
%     hold on
%     plot(InitConf(:,2),InitConf(:,3),'o')
% for i = 1:size(DirTrans,1)
%     quiver(InitConf(i,2),InitConf(i,3),10*V(i,1),10*V(i,2));
% %     quiver(InitConf(i,2),InitConf(i,3),DirTrans(i,1),DirTrans(i,2));
% end
% 
% % InitConf = [1 3 0;2 0 1;3 0 -1;4 -1 0];
% % InitConf = [1 -2 -4;2 -5 -2;3 -4 2;4 0 -2;5 4 -3;6 -1 4;7 6 3;8 6 -2;9 3 4];
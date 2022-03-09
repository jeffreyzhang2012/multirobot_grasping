classdef model
    %MODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        n_robot;
        n_side;
        object_temp;
        object;
        robot_attach_temp;
        robot_attach;
        robot_locations_temp;
        robot_locations;
        COM_temp;
        COM;
        COM_with_error;
        COM_with_error_temp;
        pose; %pose of actual COM (state X) x, y, theta
        pose_goal;
        H;
        x_traj;
        y_traj;
        th_traj;
        hist;
        x_traj_goal;
        y_traj_goal;
        th_traj_goal;
        hist_goal;
        arrow;
        % dynamics related properties
        M; % payload mass
        J; % payload inertia
        F; % robot applied forces, shape = (n_robot x 2)
        T; % robot applied torques array, shape = (n_robot x 2)
        mu0; % static friction coeff
        mu1; % kinematic friction coeff
        g; % gravity accel
        velocity; % object velocition (x.dot, y.dot, theta.dot)
        r; % COM to robot

    end
    
    methods(Static)
        function H = H_calc(pose)
            x = pose(1);
            y = pose(2);
            th = pose(3);
            H = [cos(th),-sin(th),x;sin(th),cos(th),y;0,0,1];
        end
        
        function transformed = transform(H,x)
            x = [x;ones(size(x(1,:)))];
            transformed = H * x;
            transformed = transformed(1:2,:);
        end
        function transformed = transform_vector(H,x)
            x = [x;zeros(size(x(1,:)))];
            transformed = H * x;
            transformed = transformed(1:2,:);
        end
    end
    
    methods
        function obj = model(M, J, mu0, mu1, g, vel, pose)
            load('configuration.mat');
            obj.n_robot = n_robot;
            obj.n_side = n_side;
            obj.object_temp = object;
            obj.robot_attach_temp = robot_attach;
            obj.robot_locations_temp = robot_locations;
            obj.COM_temp = [0;0];
            obj.COM_with_error_temp = COM_with_error;
            obj.H = eye(3);
            period = 5;
            obj.x_traj = @(t)3*t;
            obj.y_traj = @(t)3*sin(period*t)+3*t;
            obj.th_traj = @(t)t/5;
            obj.x_traj_goal = @(t)4*t;
            obj.y_traj_goal = @(t)3*sin(period*t)+3*t;
            obj.th_traj_goal = @(t)t/5;
            obj.hist = [0;0;0];
            obj.hist_goal = [0;0;0];
            obj.M = M;
            obj.J = J;
            obj.mu0 = mu0;
            obj.mu1 = mu1;
            obj.g = g;
            obj.velocity = vel;
            obj.r = COM_to_robots.'; % shape = n_robot x 2
            obj.pose = pose;
        end
        
        function obj = update(obj,t)
            % replace these with actual controls
            obj.pose(1) = obj.x_traj(t);
            obj.pose(2) = obj.y_traj(t);
            obj.pose(3) = obj.th_traj(t);
            % intended pose
            obj.pose_goal(1) = obj.x_traj_goal(t);
            obj.pose_goal(2) = obj.y_traj_goal(t);
            obj.pose_goal(3) = obj.th_traj_goal(t);
            obj.H = model.H_calc(obj.pose);
            obj.object = model.transform(obj.H,obj.object_temp);
            obj.robot_locations = model.transform(obj.H,obj.robot_locations_temp);
            obj.robot_attach = model.transform(obj.H,obj.robot_attach_temp);
            obj.COM = model.transform(obj.H,obj.COM_temp);
            obj.COM_with_error = model.transform(obj.H,obj.COM_with_error_temp);
            obj.arrow = model.transform_vector(obj.H,eye(2)*10);
            obj.hist = [obj.hist, obj.pose'];
            obj.hist_goal = [obj.hist_goal, obj.pose_goal'];
        end

        function obj = dynamic_update(obj, dt, t)
            % TODO: replace obj.F, obj.T with actual controls
            obj.F = zeros(obj.n_robot, 2);
            obj.F(:,1) = 10;
            obj.F(:,2) = 10;
            obj.T = zeros(obj.n_robot, 1);
            % update vel and pose
            obj = obj.payload_dynamics(dt);
            % intended pose
            obj.pose_goal(1) = obj.x_traj_goal(t);
            obj.pose_goal(2) = obj.y_traj_goal(t);
            obj.pose_goal(3) = obj.th_traj_goal(t);
            obj.H = model.H_calc(obj.pose);
            obj.object = model.transform(obj.H,obj.object_temp);
            obj.robot_locations = model.transform(obj.H,obj.robot_locations_temp);
            obj.robot_attach = model.transform(obj.H,obj.robot_attach_temp);
            obj.COM = model.transform(obj.H,obj.COM_temp);
            obj.COM_with_error = model.transform(obj.H,obj.COM_with_error_temp);
            obj.arrow = model.transform_vector(obj.H,eye(2)*10);
            obj.hist = [obj.hist, obj.pose'];
            obj.hist_goal = [obj.hist_goal, obj.pose_goal'];
        end

        function obj = payload_dynamics(obj, dt)
            % Translation
            if norm(obj.velocity(1:2)) == 0
                Ffriction = [0 0];
            else
                Ffriction = - (obj.mu0 * obj.M * obj.g) .* (obj.velocity(1:2)./norm(obj.velocity(1:2))) ...
                           - obj.mu1 .* obj.velocity(1:2);
            end
            total_force = sum(obj.F, 1) + Ffriction;
            accel = total_force./obj.M;
            % Rotational
            Tfriction = - (obj.mu1 * obj.J * obj.velocity(3) / obj.M); % only consider viscous (kinematic)
            % T = sum (T_i + ri x Fi) + Tfriction;
            total_torque = sum(obj.T, 1) + sum (obj.r(:,1).*obj.F(:,2)-obj.r(:,2).*obj.F(:,1)) + Tfriction;
            alpha = total_torque/obj.J;
            % update object velocity
            obj.velocity(1:2)  = obj.velocity(1:2) + accel.*dt;
            obj.velocity(3) = obj.velocity(3) + alpha*dt;
            % update object pose
            obj.pose(1:2) = obj.pose(1:2) + obj.velocity(1:2).*dt;
            obj.pose(3) = obj.pose(3) + obj.velocity(3)*dt;
        end

        
        function draw(obj)
            plot(polyshape(obj.object(1,:),obj.object(2,:)),'DisplayName','Object');
            title("Press q to quit early");
            for i = 1:obj.n_robot
                line([obj.robot_locations(1,i), obj.robot_attach(1,i)],[obj.robot_locations(2,i), obj.robot_attach(2,i)],'HandleVisibility','off');
            end
            hold on;
            scatter(obj.robot_locations(1,:),obj.robot_locations(2,:),'MarkerEdgeColor',[0 .5 .5],'MarkerFaceColor',[0 .7 .7],'LineWidth',1.5,'DisplayName','Robots');
            scatter(obj.COM(1),obj.COM(2),'DisplayName','com');
            scatter(obj.COM_with_error(1),obj.COM_with_error(2),'DisplayName','Erroneous com');
            quiver(obj.COM(1),obj.COM(2),obj.arrow(1,1),obj.arrow(2,1),0, 'MaxHeadSize',0.5,'HandleVisibility','off')
            quiver(obj.COM(1),obj.COM(2),obj.arrow(1,2),obj.arrow(2,2),0, 'MaxHeadSize',0.5,'HandleVisibility','off')
            plot(obj.hist(1,:),obj.hist(2,:),'Color','k','LineStyle','--','DisplayName','Object Trajectory');
            plot(obj.hist_goal(1,:),obj.hist_goal(2,:),'Color','r','LineStyle','--','DisplayName','Goal Trajectory');
            hold off;
        end
    end
end


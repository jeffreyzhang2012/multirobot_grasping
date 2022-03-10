classdef kinematic_controller
    properties
        n_robot;
        M;
        g;
        mu0;
        mu1;
        % proportional control coeffs
        Kf;     % force proportional control
        Kt;     % torque proportional control
        EPS;    % a small value for numerical stability
        v;
        w;
        % desired velocities and weights
        vd;
        vd_w;
        vd_n;   % normal component
        vd_wn;  % weight of normal component
        vd_t;   % tangential component
        vd_wt;  % weight of tangential component
        wd;
        wd_k;
    end
        
    methods
        %% initialize
        function obj = kinematic_controller(model, t)
            obj.n_robot = model.n_robot;
            obj.M = model.M;
            obj.g = model.g;
            obj.mu0 = model.mu0;
            obj.mu1 = model.mu1;
            obj.Kf = 1;
            obj.Kt = 2;
            obj.EPS = 1e-9;
            obj.v = model.velocity(1:2);
            obj.w = model.velocity(3);
            obj.vd_w = 1;
            obj.vd_wn = 20;
            obj.vd_wt = 10;
            obj.wd_k = 1;
            obj = obj.compute_desired_vel(model, t);
        end
        
        function obj = compute_desired_vel(obj, model, t)
        % The functionality of this function should be realized in
        % dynamics, abandon this when we have nice vd and wd
            
            % compute desired velocities
            xc = zeros(1, 2);
            xa = zeros(1, 2);
            xc(1) = model.pose(1);
            xc(2) = model.pose(2);
            xa(1) = model.x_traj_goal(t);
            xa(2) = model.y_traj_goal(t);
            obj.vd = obj.vd_w .* (xa - xc);

            % theta_d is the desired theta
            theta_d = atan2(model.y_traj_goal(t) - model.pose(2),...
                model.x_traj_goal(t) - model.pose(1));
            theta = model.pose(3);
            obj.wd = obj.wd_k * (theta_d - theta);
        end
        
        %% leader controller
        function [F, T] = leader(obj, vd, wd)
        % This function simulates the leader controller outputs.
            mag = obj.Kf;
            dir = vd - obj.v;
            fprintf("obj.v %s\n", mat2str(obj.v))
            fprintf("dir %s\n", mat2str(dir))
            mag_comp = obj.mu0 * obj.M * obj.g / obj.n_robot;
            dir_comp = obj.v ./ (norm(obj.v) + obj.EPS);
            F_vd = mag .* dir;
            F_comp = mag_comp .* dir_comp;
            fprintf('F_comp = %s\n', mat2str(F_comp))
            fprintf('F_vd = %s\n', mat2str(F_vd))
            F = F_vd + F_comp;
            
            % TODO: test consensus using fixed leader force
            % all follower forces should converge to this force
%             F = [5, 0];
            T = obj.Kt * (wd - obj.w);
%             fprintf("leader mag = %.2f\nleader dir = [%.2f, %.2f]\n",...
%                 mag, dir(1), dir(2));
        end
        
        %% follower controller
        function [F, T] = follower(obj, acc, F_prev, F_leader)
            % input:
            %   acc - object acceleration
            %   F_prev - force applied by robot i at previous timestep,
            %   this is required because the consensus outputs the update
            acc(1) = acc(1) * (1 + 0.01 * (2 * rand - 1));
            acc(2) = acc(2) * (1 + 0.01 * (2 * rand - 1));
            acc_term = obj.M .* acc;
            fprintf('acc_term %s\n', mat2str(acc_term))
            friction_term = ...
                obj.mu0 * obj.M * obj.g .* obj.v ./ (norm(obj.v) + obj.EPS)...
                + obj.mu1 .* obj.v;
            fprintf('friction_term %s\n', mat2str(friction_term))
            robot_term = obj.n_robot .* F_prev;
            fprintf('robot %s\n', mat2str(robot_term))
            Fdot = acc_term + friction_term - robot_term;
            % idea consensus
%             Fdot = F_leader - F_prev;
            F = F_prev + Fdot;% * .1;
            T = 0;
%             fprintf("follower mag = %.2f\nfollower dir = [%.2f, %.2f]\n",...
%                 mag, dir(1), dir(2));

        end
    end
    
end
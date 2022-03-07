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
            obj.Kf = 5; % worked with 50 for leader only
            obj.Kt = 10; % worked with 10 for leader only
            obj.EPS = 1e-9;
            obj.v = model.velocity(1:2);
            obj.w = model.velocity(3);
            obj.vd_w = 15; % worked with 15 for leader only
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
%             Following vn and vt are from the paper but doesn't fit
%             in our trajectory
            % vn is the normal component (current pos -> goal)
            obj.vd_n = (xa - xc) ./ (norm(xa - xc) + obj.EPS);
            % vt is the tangential component at desired traj
            vt = zeros(1, 2);
            vt(1) = 3;
            vt(2) = 15 * sin(5*t) + 3;
            obj.vd_t = vt ./ norm(vt);
%             obj.vd = obj.vd_wn .* vn + obj.vd_wt .* vt;
            obj.vd = obj.vd_w .* (xa - xc) ./ (norm(xa - xc) + obj.EPS);
            fprintf('obj.v = %s\n', mat2str(obj.v))
            fprintf('obj.vd = %s\n', mat2str(obj.vd))
            % theta_d is the angle along the trajectory
            theta_d = 5 * sin(5 * t) + 1;
            theta = model.pose(3);
            obj.wd = obj.wd_k * (theta_d - theta);
        end
        
        %% leader controller
        function [F, T] = leader(obj, vd, wd)
        % This function simulates the leader controller outputs.
            
%             % old controller
%             mag = obj.Kf * max(norm(obj.vd) - norm(obj.v), 0);
%             dir = obj.vd ./ (norm(obj.vd) + obj.EPS);

            % new controller
            mag = obj.Kf;
            dir = vd - obj.v;
            mag_comp = obj.mu0 * obj.g / obj.n_robot;
            dir_comp = obj.v ./ (norm(obj.v) + obj.EPS);
            F_vd = mag .* dir;
            F_comp = mag_comp .* dir_comp;
            fprintf('F_comp = %s\n', mat2str(F_comp))
            fprintf('F_vd = %s\n', mat2str(F_vd))
            F = F_vd + F_comp;
%             F = F_vd;
            T = obj.Kt * (wd - obj.w);
%             fprintf("leader mag = %.2f\nleader dir = [%.2f, %.2f]\n",...
%                 mag, dir(1), dir(2));
        end
        
        %% follower controller
        function [F, T] = follower(obj, acc, F_prev)
        % This function simulates the follower controller outputs.
        % The follower robots cannot generate any torque.
            % TODO: verify follower dynamics
            
%             % old controller
%             mag = obj.mu0 * obj.M * obj.g / obj.n_robot;
%             dir = obj.v ./ (norm(obj.v) + obj.EPS);
%             F = mag .* dir;

            % new controller
            % input:
            %   acc - object acceleration
            %   F_prev - force applied by robot i at previous timestep,
            %   this is required because the consensus outputs the update
            dF = obj.M .* acc + obj.mu0 * obj.M * obj.g .* obj.v ./ (norm(obj.v) + obj.EPS)...
                + obj.mu1 .* obj.v - obj.n_robot .* F_prev;
            F = F_prev + dF;
            T = 0;
%             fprintf("follower mag = %.2f\nfollower dir = [%.2f, %.2f]\n",...
%                 mag, dir(1), dir(2));

        end
    end
    
end
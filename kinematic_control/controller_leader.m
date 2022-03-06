function [F, T] = controller_leader(v, w, vd, wd)
% This function simulates the leader controller outputs.
    % TODO: the velocities should be in local reference frame
    global Kf, global Kt, global g, global EPS, global ref, global mu_s, global N
    if strcmp(ref, 'global')
        mag = Kf;
        dir = vd - v;
%         mag = Kp * max(norm(vd) - norm(v), 0);
%         dir = vd ./ (norm(vd) + EPS);
        mag_comp = mu_s * g / N;
        dir_comp = v ./ (norm(v) + EPS);
    elseif strcmp(ref, 'local')
        vd1 = vd - v;   % assuming v_object == v_leader
        v1 = v - v;
        mag = Kp * max(norm(vd1) - norm(v1), 0);
        dir = vd1 ./ (norm(vd1) + EPS);
        mag_comp = mu_s * g / N;
        dir_comp = v1 ./ (norm(v1) + EPS);
    end
    F = mag .* dir + mag_comp .* dir_comp;
    T = Kt * (wd - w);
%     fprintf("leader mag = %.2f\nleader dir = [%.2f, %.2f]\n",...
%         mag, dir(1), dir(2));
end
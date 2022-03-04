function F = controller_leader(v, vd)
% This function simulates the leader controller outputs.
    % TODO: the velocities should be in local reference frame
    global Kp, global EPS, global ref
    if strcmp(ref, 'global')
        mag = Kp * max(norm(vd) - norm(v), 0);
        dir = vd ./ (norm(vd) + EPS);
    elseif strcmp(ref, 'local')
        vd1 = vd - v;   % assuming v_object == v_leader
        v1 = v - v;
        mag = Kp * max(norm(vd1) - norm(v1), 0);
        dir = vd1 ./ (norm(vd1) + EPS);
    end
    F = mag .* dir;
%     fprintf("leader mag = %.2f\nleader dir = [%.2f, %.2f]\n",...
%         mag, dir(1), dir(2));
end
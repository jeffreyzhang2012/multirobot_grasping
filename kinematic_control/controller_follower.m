function F = controller_follower(v)
% This function simulates the follower controller outputs
    % TODO: verify follower dynamics
    % TODO: velocity should be in follower local ref frame
    global mu, global M, global g, global N, global EPS, global ref
    mag = mu * M * g / N;
    if strcmp(ref, 'global')
        dir = v ./ (norm(v) + EPS);
    else
        F = 0;
    end
    F = mag .* dir;
end
% noap -> joints (rad, -pi ~ pi)
function joint = noap2joint(noap, rb)
    sol = rb.inverse(noap); % all 8 solutions
    theta = rad2deg0(sol.th); % rad2deg(), but output range is in -180~180 degree
    
    joint = [];
    for i = 1 : size(theta, 2)
        if ~sol.OofR(i) % solutions that not out of reach
            if isempty(rb.isOutofRange(theta(:, i))) % solutions that not out of range
                joint = cat(2, joint, deg2rad(theta(:, i)));
            end
        end
    end
end


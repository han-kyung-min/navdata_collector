function d = quat_distance(q1, q2)
    % q1, q2 = [w x y z] or [qw qx qy qz]
    q1 = q1 / norm(q1);
    q2 = q2 / norm(q2);
    dot_val = abs(dot(q1, q2));        % handle q and -q equivalence
    dot_val = min(max(dot_val, -1.0), 1.0);
    d = 2 * acos(dot_val);
end
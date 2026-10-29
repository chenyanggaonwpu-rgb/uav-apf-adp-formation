function y = limit_vector3d(x, maxNorm)
% LIMIT_VECTOR3D Bound each column by its Euclidean norm.
    y = x;
    mag = sqrt(sum(x.^2, 1));
    active = mag > maxNorm;
    if any(active)
        y(:, active) = x(:, active) .* (maxNorm ./ mag(active));
    end
end

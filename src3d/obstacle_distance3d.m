function [sd, normal] = obstacle_distance3d(p, obstacle)
% OBSTACLE_DISTANCE3D Signed distance and outward normal for a closed solid.
% Cylinders include both caps. Boxes include faces, edges and corners.
    if strcmp(obstacle.type,'box')
        delta=p(:)-obstacle.center(:);
        q=abs(delta)-obstacle.halfSize(:);
        outside=max(q,0);
        sd=norm(outside)+min(max(q),0);
        direction=sign(delta); direction(direction==0)=1;
        if any(outside>0)
            normal=direction.*outside/norm(outside);
        else
            [~,axisIndex]=max(q);
            normal=zeros(3,1); normal(axisIndex)=direction(axisIndex);
        end
        return;
    end
    if strcmp(obstacle.type,'sphere')
        d = p - obstacle.center(:);
        length = norm(d);
        sd = length - obstacle.r;
        normal = d / max(length,eps);
        if length < 1e-12, normal = [1;0;0]; end
        return;
    end
    if ~strcmp(obstacle.type,'cylinder')
        error('UAV3D:ObstacleType','Unsupported obstacle type.');
    end
    xy = p(1:2) - obstacle.center(:);
    radial = norm(xy);
    radialNormal = xy / max(radial,eps);
    if radial < 1e-12, radialNormal = [1;0]; end
    mid = (obstacle.zmin + obstacle.zmax)/2;
    halfHeight = (obstacle.zmax - obstacle.zmin)/2;
    qr = radial - obstacle.r;
    qz = abs(p(3)-mid) - halfHeight;
    outside = max([qr;qz],0);
    sd = norm(outside) + min(max(qr,qz),0);
    zSign = sign(p(3)-mid);
    if zSign==0, zSign=1; end
    if qr>0 && qz>0
        normal = [radialNormal*qr;zSign*qz];
        normal = normal/norm(normal);
    elseif qr >= qz
        normal = [radialNormal;0];
    else
        normal = [0;0;zSign];
    end
end

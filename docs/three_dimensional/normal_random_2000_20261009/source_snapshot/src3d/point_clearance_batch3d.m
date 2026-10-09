function distances=point_clearance_batch3d(points,obstacles)
% POINT_CLEARANCE_BATCH3D Signed distance to the obstacle union, one value per point.
    distances=inf(1,size(points,2));
    for o=obstacles
        if strcmp(o.type,'sphere')
            d=vecnorm(points-o.center(:))-o.r;
        elseif strcmp(o.type,'cylinder')
            qr=vecnorm(points(1:2,:)-o.center(:))-o.r;
            qz=abs(points(3,:)-(o.zmin+o.zmax)/2)-(o.zmax-o.zmin)/2;
            d=hypot(max(qr,0),max(qz,0))+min(max(qr,qz),0);
        elseif strcmp(o.type,'box')
            q=abs(points-o.center(:))-o.halfSize(:);
            d=vecnorm(max(q,0))+min(max(q,[],1),0);
        else
            error('UAV3D:ObstacleType','Unsupported obstacle type: %s',o.type);
        end
        distances=min(distances,d);
    end
end

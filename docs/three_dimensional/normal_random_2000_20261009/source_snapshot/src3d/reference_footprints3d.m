function footprints=reference_footprints3d(obstacles,height,h,v)
% REFERENCE_FOOTPRINTS3D XY slices of solids dilated by an XY/Z ellipsoid.
% A common synchronized reference height permits one slice calculation per
% cycle. Sphere slices are sums of circular and ellipsoidal cross sections.
    footprints=struct('type',{},'center',{},'half',{},'padding',{});
    for o=obstacles
        if strcmp(o.type,'sphere')
            dz=abs(height-o.center(3));
            if dz>=o.r+v, continue; end
            if h==v
                padding=sqrt(max(0,(o.r+h)^2-dz^2));
            else
                lo=max(-o.r,dz-v); hi=min(o.r,dz+v);
                fn=@(t) sqrt(max(0,o.r^2-t.^2))+ ...
                    h*sqrt(max(0,1-((dz-t)/v).^2));
                [at,negative]=fminbnd(@(t) -fn(t),lo,hi, ...
                    optimset('TolX',1e-7,'Display','off'));
                padding=max([fn(lo),fn(hi),fn(at),-negative])+1e-7;
            end
            f=struct('type','circle','center',o.center(1:2), ...
                'half',zeros(2,1),'padding',padding);
        else
            dz=max([o.zmin-height,height-o.zmax,0]);
            if dz>=v, continue; end
            padding=h*sqrt(max(0,1-(dz/v)^2));
            if strcmp(o.type,'cylinder')
                f=struct('type','circle','center',o.center(:), ...
                    'half',zeros(2,1),'padding',o.r+padding);
            elseif strcmp(o.type,'box')
                f=struct('type','box','center',o.center(1:2), ...
                    'half',o.halfSize(1:2),'padding',padding);
            else
                error('UAV3D:ObstacleType','Unsupported obstacle type.');
            end
        end
        footprints(end+1)=f; %#ok<AGROW>
    end
end

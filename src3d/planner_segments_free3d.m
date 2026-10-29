function [free,stats]=planner_segments_free3d(a,b,obstacles,clearance)
% PLANNER_SEGMENTS_FREE3D Fast certificates, with the original check at the boundary.
% For each convex solid, any unit normal defines a separating support plane.
% Bisection only finds useful normals; it never substitutes an approximate
% distance for an unresolved segment. The legacy scalar minimizer decides
% close cases, preserving the planner's original numerical tolerance.
    if size(a,1)~=3 || ~isequal(size(a),size(b))
        error('UAV3D:PlannerSegments','Expected matching 3-by-N endpoints.');
    end
    validateattributes(clearance,{'numeric'},{'scalar','finite','nonnegative'});
    count=size(a,2);free=true(1,count);
    stats=struct('segments',count,'fallbacks',0,'obstacleChecks',0);
    delta=b-a;lengths=vecnorm(delta);
    % fminbnd uses TolX=1e-6 in the legacy oracle. Do not decide a rejection
    % from a new interior sample inside its spatial uncertainty band.
    band=1e-9+1e-5*lengths;
    fallback=false(1,count);
    for o=obstacles
        pending=find(free & ~fallback);
        if isempty(pending),break;end
        stats.obstacleChecks=stats.obstacleChecks+numel(pending);
        if strcmp(o.type,'cylinder')
            center=[o.center(:);(o.zmin+o.zmax)/2];
        else
            center=o.center(:);
        end
        if strcmp(o.type,'sphere')
            t=max(0,min(1,sum((center-a(:,pending)).*delta(:,pending),1)./ ...
                max(sum(delta(:,pending).^2,1),eps)));
            distance=vecnorm(a(:,pending)+delta(:,pending).*t-center)-o.r;
            close=abs(distance-clearance)<=band(pending);
            free(pending(distance<clearance & ~close))=false;
            fallback(pending(close))=true;
            continue;
        end
        lo=zeros(1,numel(pending));hi=ones(size(lo));
        t=max(0,min(1,sum((center-a(:,pending)).*delta(:,pending),1)./ ...
            max(sum(delta(:,pending).^2,1),eps)));
        for iteration=1:22
            if isempty(pending),break;end
            sample=a(:,pending)+delta(:,pending).*t;
            [distance,normal]=solid_distance_normal(sample,o);
            if strcmp(o.type,'box')
                support=sum(abs(normal).*o.halfSize(:),1);
            else
                support=o.r*vecnorm(normal(1:2,:))+ ...
                    (o.zmax-o.zmin)/2*abs(normal(3,:));
            end
            gap=min(sum(normal.*(a(:,pending)-center),1), ...
                sum(normal.*(b(:,pending)-center),1))-support;
            bad=distance<clearance-band(pending);
            certified=gap>=clearance+1e-9 & ~bad;
            free(pending(bad))=false;
            derivative=sum(normal.*delta(:,pending),1);
            hi(derivative>0)=t(derivative>0);
            lo(derivative<=0)=t(derivative<=0);
            keep=~bad & ~certified;
            pending=pending(keep);lo=lo(keep);hi=hi(keep);t=(lo+hi)/2;
        end
        fallback(pending)=true;
    end
    for id=find(fallback & free)
        free(id)=segment_clearance3d(a(:,id),b(:,id),obstacles)>=clearance;
        stats.fallbacks=stats.fallbacks+1;
    end
    free=free(:);
end

function [distance,normal]=solid_distance_normal(points,o)
    if strcmp(o.type,'box')
        delta=points-o.center(:);q=abs(delta)-o.halfSize(:);
        outside=max(q,0);magnitude=vecnorm(outside);
        [insideDistance,axisIndex]=max(q,[],1);
        distance=magnitude+min(insideDistance,0);
        direction=sign(delta);direction(direction==0)=1;
        normal=direction.*outside./max(magnitude,eps);
        inside=find(magnitude==0);
        normal(sub2ind(size(normal),axisIndex(inside),inside))= ...
            direction(sub2ind(size(direction),axisIndex(inside),inside));
    elseif strcmp(o.type,'cylinder')
        xy=points(1:2,:)-o.center(:);radial=vecnorm(xy);
        radialNormal=xy./max(radial,eps);
        radialNormal(:,radial<1e-12)=repmat([1;0],1,nnz(radial<1e-12));
        z=points(3,:)-(o.zmin+o.zmax)/2;
        qr=radial-o.r;qz=abs(z)-(o.zmax-o.zmin)/2;
        positive=max([qr;qz],0);
        distance=vecnorm(positive)+min(max(qr,qz),0);
        zSign=sign(z);zSign(zSign==0)=1;
        normal=zeros(3,size(points,2));
        corner=qr>0 & qz>0;side=~corner & qr>=qz;cap=~corner & ~side;
        if any(corner)
            normal(:,corner)=[radialNormal(:,corner).*qr(corner);zSign(corner).*qz(corner)];
            normal(:,corner)=normal(:,corner)./vecnorm(normal(:,corner));
        end
        normal(1:2,side)=radialNormal(:,side);normal(3,cap)=zSign(cap);
    else
        error('UAV3D:ObstacleType','Unsupported obstacle type: %s',o.type);
    end
end

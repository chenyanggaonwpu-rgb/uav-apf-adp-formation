function [distance,normals]=directional_clearance3d(points,obstacles,margins)
% DIRECTIONAL_CLEARANCE3D Distance in coordinates scaled by XY/Z margins.
% A value >=1 excludes the Minkowski sum of each solid and a margin
% ellipsoid. Physical collision distances remain unscaled elsewhere.
% Sphere becomes an ellipsoid; its exterior distance uses a bracketed
% projection. The lower bracket yields a conservative numerical distance.
    margins=margins(:);
    distance=inf(1,size(points,2));
    normals=zeros(size(points)); needNormal=nargout>1;
    h=margins(1); v=margins(3);
    for o=obstacles
        if strcmp(o.type,'box')
            q=(abs(points-o.center(:))-o.halfSize(:))./margins;
            d=sqrt(sum(max(q,0).^2,1))+min(max(q,[],1),0);
            if needNormal
                normal=sign(points-o.center(:)).*max(q,0);
                inside=vecnorm(normal)<1e-12;
                [~,axisIndex]=max(q(:,inside),[],1);
                ids=find(inside);
                for j=1:numel(ids), normal(axisIndex(j),ids(j))=1; end
                normal=normal./max(vecnorm(normal),eps);
            end
        elseif strcmp(o.type,'cylinder')
            radial=(vecnorm(points(1:2,:)-o.center(:))-o.r)/h;
            vertical=max(o.zmin-points(3,:),points(3,:)-o.zmax)/v;
            q=[radial;vertical];
            d=sqrt(sum(max(q,0).^2,1))+min(max(q,[],1),0);
            if needNormal
                radialDirection=(points(1:2,:)-o.center(:))./max(vecnorm(points(1:2,:)-o.center(:)),eps);
                signZ=sign(points(3,:)-(o.zmin+o.zmax)/2); signZ(signZ==0)=1;
                normal=[radialDirection.*max(radial,0);signZ.*max(vertical,0)];
                inside=vecnorm(normal)<1e-12;
                side=inside & radial>=vertical; cap=inside & ~side;
                normal(1:2,side)=radialDirection(:,side); normal(3,cap)=signZ(cap);
                normal=normal./max(vecnorm(normal),eps);
            end
        elseif strcmp(o.type,'sphere')
            delta=points-o.center(:);
            if h==v
                d=(vecnorm(delta)-o.r)/h;
                if needNormal, normal=delta./max(vecnorm(delta),eps); end
            else
                rho=vecnorm(delta(1:2,:)); z=abs(delta(3,:));
                inside=rho.^2+z.^2<=o.r^2;
                d=(sqrt(rho.^2+z.^2)/o.r-1)*min(o.r/h,o.r/v);
                q1=rho(~inside)/h; q2=z(~inside)/v;
                a1=o.r/h; a2=o.r/v;
                low=zeros(size(q1)); high=sqrt((a1*q1).^2+(a2*q2).^2);
                for iteration=1:38
                    t=(low+high)/2;
                    value=(a1*q1./(t+a1^2)).^2+(a2*q2./(t+a2^2)).^2;
                    left=value>1;
                    low(left)=t(left); high(~left)=t(~left);
                end
                d(~inside)=sqrt((q1.*low./(low+a1^2)).^2+ ...
                    (q2.*low./(low+a2^2)).^2);
                if needNormal
                    normal=delta./margins;
                    radialDirection=delta(1:2,~inside)./max(rho(~inside),eps);
                    normal(:,~inside)=[radialDirection.*(q1.*low./(low+a1^2)); ...
                        sign(delta(3,~inside)).*(q2.*low./(low+a2^2))];
                    normal=normal./max(vecnorm(normal),eps);
                end
            end
        else
            error('UAV3D:ObstacleType','Unsupported obstacle type.');
        end
        if needNormal
            keep=d<distance; normals(:,keep)=normal(:,keep);
        end
        distance=min(distance,d);
    end
end

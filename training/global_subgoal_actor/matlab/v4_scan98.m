function [scan,directions]=v4_scan98(position,obstacles,workspace,rangeMeters)
% Vectorized first hits on the same sphere/closed-cylinder/AABB geometry.
% The first 26 directions match adp3d_scan26 exactly; no occupancy oracle.
    persistent rays
    if isempty(rays)
        [gx,gy,gz]=ndgrid(-1:1,-1:1,-1:1);grid=[gx(:),gy(:),gz(:)]';
        axes=[1,-1,0,0,0,0;0,0,1,-1,0,0;0,0,0,0,1,-1];
        old=[axes,grid(:,sum(grid~=0,1)>1)];old=old./vecnorm(old,2,1);
        k=0:71;z=1-2*(k+.5)/72;az=pi*(3-sqrt(5))*k;r=sqrt(1-z.^2);
        rays=[old,[r.*cos(az);r.*sin(az);z]];
    end
    directions=rays;origin=position(:);assert(isequal(size(origin),[3 1]));
    assert(all(origin>=workspace(1,:)'-1e-8)&&all(origin<=workspace(2,:)'+1e-8));
    distance=rangeMeters*ones(1,98);
    for axis=1:3
        d=rays(axis,:);candidate=inf(1,98);positive=d>1e-12;negative=d< -1e-12;
        candidate(positive)=(workspace(2,axis)-origin(axis))./d(positive);
        candidate(negative)=(workspace(1,axis)-origin(axis))./d(negative);
        distance=min(distance,max(0,candidate));
    end
    bounds=dv2_obstacle_bounds(obstacles);
    obstacles=dv2_near_obstacles(origin,obstacles,bounds,rangeMeters+1e-8);
    for obstacle=obstacles
        switch obstacle.type
            case 'box'
                lo=obstacle.center(:)-obstacle.halfSize(:);hi=obstacle.center(:)+obstacle.halfSize(:);
                enter=zeros(1,98);exit=inf(1,98);
                for axis=1:3
                    d=rays(axis,:);moving=abs(d)>=1e-12;
                    first=-inf(1,98);last=inf(1,98);
                    a=(lo(axis)-origin(axis))./d(moving);b=(hi(axis)-origin(axis))./d(moving);
                    first(moving)=min(a,b);last(moving)=max(a,b);
                    if origin(axis)<lo(axis)||origin(axis)>hi(axis),first(~moving)=inf;last(~moving)=-inf;end
                    enter=max(enter,first);exit=min(exit,last);
                end
                enter(enter>exit|exit<0)=inf;distance=min(distance,max(0,enter));
            case 'sphere'
                delta=origin-obstacle.center(:);a=sum(rays.^2,1);b=2*(delta'*rays);c=dot(delta,delta)-obstacle.r^2;
                if c<=0,hit=zeros(1,98);
                else
                    disc=b.^2-4*a*c;tolerance=1e-12*max(1,b.^2+abs(4*a*c));valid=disc>=-tolerance;
                    t1=(-b-sqrt(max(0,disc)))./(2*a);t2=(-b+sqrt(max(0,disc)))./(2*a);
                    t1(~valid|t1<0)=inf;t2(~valid|t2<0)=inf;hit=min(t1,t2);
                end
                distance=min(distance,hit);
            case 'cylinder'
                xy=origin(1:2)-obstacle.center(:);
                if dot(xy,xy)<=obstacle.r^2&&origin(3)>=obstacle.zmin&&origin(3)<=obstacle.zmax
                    hit=zeros(1,98);
                else
                    hit=inf(1,98);a=sum(rays(1:2,:).^2,1);b=2*(xy'*rays(1:2,:));c=dot(xy,xy)-obstacle.r^2;
                    disc=b.^2-4*a*c;tolerance=1e-12*max(1,b.^2+abs(4*a*c));valid=a>1e-14&disc>=-tolerance;
                    for sign=[-1 1]
                        t=(-b+sign*sqrt(max(0,disc)))./(2*a);height=origin(3)+t.*rays(3,:);
                        t(~valid|t<0|height<obstacle.zmin|height>obstacle.zmax)=inf;hit=min(hit,t);
                    end
                    for cap=[obstacle.zmin obstacle.zmax]
                        t=(cap-origin(3))./rays(3,:);radial=xy+rays(1:2,:).*t;
                        valid=abs(rays(3,:))>1e-12&t>=0&sum(radial.^2,1)<=obstacle.r^2+1e-12*max(1,obstacle.r^2);
                        t(~valid)=inf;hit=min(hit,t);
                    end
                end
                distance=min(distance,hit);
            otherwise,error('V4:ObstacleType','Unsupported solid: %s',obstacle.type);
        end
    end
    scan=single(max(0,min(1,distance(:)/rangeMeters)));
end

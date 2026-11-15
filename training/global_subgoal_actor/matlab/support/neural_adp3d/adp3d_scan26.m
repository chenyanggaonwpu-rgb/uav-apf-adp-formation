function [scan, info] = adp3d_scan26(position, obstacles, workspace, rangeMeters)
% ADP3D_SCAN26 用三维实体和工作空间边界生成 26 束理想局部测距。
% 输出 scan 为 26x1，每项为中心测距/rangeMeters；数值范围 [0,1]。
% 第 1~6 束分别朝 +x、-x、+y、-y、+z、-z，其余为面/体对角线方向。
% 球、闭合竖直圆柱、盒体均求量程内第一次命中，包含上下盖面。
% 世界坐标方向固定。本函数是仿真传感器；未加入噪声、姿态和时延。
% 26 条射线之间仍有盲区，测距不是独立的碰撞安全保证。

    validateattributes(position, {'numeric'}, {'real','finite','size',[3,1]});
    validateattributes(workspace, {'numeric'}, {'real','finite','size',[2,3]});
    validateattributes(rangeMeters, {'numeric'}, {'real','finite','scalar','positive'});
    assert(all(workspace(2,:)>workspace(1,:)), 'ADP3D:Workspace','工作空间上下界无效。');
    assert(all(position>=workspace(1,:)') && all(position<=workspace(2,:)'), ...
        'ADP3D:PositionOutside','测距位置必须位于工作空间内。');

    [gx,gy,gz] = ndgrid(-1:1,-1:1,-1:1);
    gridDirections = [gx(:),gy(:),gz(:)]';
    diagonals = gridDirections(:,sum(gridDirections~=0,1)>1);
    axisDirections = [1,-1,0,0,0,0; 0,0,1,-1,0,0; 0,0,0,0,1,-1];
    directions = [axisDirections,diagonals];
    directions = directions./vecnorm(directions,2,1);
    distances = rangeMeters*ones(26,1);
    hitMask = false(26,1);
    hitKind = repmat("range",26,1);
    obstacleIndex = zeros(26,1);

    for beam = 1:26
        direction = directions(:,beam);
        boundaryDistance = rayWorkspaceExit(position,direction,workspace);
        if boundaryDistance<=distances(beam)
            distances(beam) = boundaryDistance;
            hitMask(beam) = true;
            hitKind(beam) = "boundary";
        end
        for j = 1:numel(obstacles)
            candidate = raySolidHit(position,direction,obstacles(j));
            if candidate<=distances(beam)
                distances(beam) = max(0,candidate);
                hitMask(beam) = true;
                hitKind(beam) = "obstacle";
                obstacleIndex(beam) = j;
            end
        end
    end
    scan = distances/rangeMeters;
    info = struct('directions',directions, 'distancesMeters',distances, ...
        'rangeMeters',rangeMeters, 'hitMask',hitMask, 'hitKind',hitKind, ...
        'obstacleIndex',obstacleIndex, ...
        'endpoints',position+directions.*distances', ...
        'axisBeamIndices',struct('plusX',1,'minusX',2,'plusY',3,'minusY',4,'plusZ',5,'minusZ',6));
end

function distance = rayWorkspaceExit(origin,direction,workspace)
    distance = inf;
    for axisIndex = 1:3
        if direction(axisIndex)>1e-12
            candidate = (workspace(2,axisIndex)-origin(axisIndex))/direction(axisIndex);
        elseif direction(axisIndex)<-1e-12
            candidate = (workspace(1,axisIndex)-origin(axisIndex))/direction(axisIndex);
        else
            continue;
        end
        distance = min(distance,max(0,candidate));
    end
end

function distance = raySolidHit(origin,direction,solid)
    switch solid.type
        case 'box'
            lower = solid.center(:)-solid.halfSize(:);
            upper = solid.center(:)+solid.halfSize(:);
            entry = 0;
            exit = inf;
            for axisIndex = 1:3
                if abs(direction(axisIndex))<1e-12
                    if origin(axisIndex)<lower(axisIndex) || origin(axisIndex)>upper(axisIndex)
                        distance = inf; return;
                    end
                else
                    limits = sort(([lower(axisIndex),upper(axisIndex)]-origin(axisIndex))/direction(axisIndex));
                    entry = max(entry,limits(1));
                    exit = min(exit,limits(2));
                    if entry>exit
                        distance = inf; return;
                    end
                end
            end
            distance = entry;
        case 'sphere'
            delta = origin-solid.center(:);
            a = dot(direction,direction);
            b = 2*dot(delta,direction);
            c = dot(delta,delta)-solid.r^2;
            if c<=0
                distance = 0; return;
            end
            discriminant = b^2-4*a*c;
            tolerance = 1e-12*max(1,b^2+abs(4*a*c));
            if discriminant < -tolerance
                distance = inf; return;
            end
            roots = sort((-b+[-1,1]*sqrt(max(0,discriminant)))/(2*a));
            candidates = roots(roots>=0);
            if isempty(candidates), distance=inf; else, distance=min(candidates); end
        case 'cylinder'
            xy = origin(1:2)-solid.center(:);
            if dot(xy,xy)<=solid.r^2 && origin(3)>=solid.zmin && origin(3)<=solid.zmax
                distance = 0; return;
            end
            distance = inf;
            a = dot(direction(1:2),direction(1:2));
            b = 2*dot(xy,direction(1:2));
            c = dot(xy,xy)-solid.r^2;
            discriminant = b^2-4*a*c;
            tolerance = 1e-12*max(1,b^2+abs(4*a*c));
            if a>1e-14 && discriminant>=-tolerance
                roots = (-b+[-1,1]*sqrt(max(0,discriminant)))/(2*a);
                for candidate = roots
                    height = origin(3)+candidate*direction(3);
                    if candidate>=0 && height>=solid.zmin && height<=solid.zmax
                        distance = min(distance,candidate);
                    end
                end
            end
            if abs(direction(3))>1e-12
                for cap = [solid.zmin,solid.zmax]
                    candidate = (cap-origin(3))/direction(3);
                    radial = xy+candidate*direction(1:2);
                    if candidate>=0 && dot(radial,radial)<=solid.r^2+1e-12*max(1,solid.r^2)
                        distance = min(distance,candidate);
                    end
                end
            end
        otherwise
            error('ADP3D:UnsupportedSolid','不支持的三维实体：%s',solid.type);
    end
end

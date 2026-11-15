function [waypoints,info,cache] = plan_fixed_layers3d(startPoint,goalPoint,obstacles,p,clearance,startJoinClearance,cache,buildOnly)
% PLAN_FIXED_LAYERS3D Finite horizontal 8-neighbor grids plus vertical-only links.
% Every graph edge, start/goal connector and simplified segment is checked.
    if nargin<5, clearance=p.pathClearance; end
    if nargin<6, startJoinClearance=clearance; end
    if nargin<7, cache=[]; end
    if nargin<8, buildOnly=false; end
    if isfield(p,'directionalClearance') && p.directionalClearance.enable
        [waypoints,info,cache]=plan_directional_layers3d(startPoint,goalPoint, ...
            obstacles,p,clearance,startJoinClearance,cache,buildOnly);
        return;
    end
    verticalWeight=0;
    if isfield(p,'verticalCostWeight'), verticalWeight=p.verticalCostWeight; end
    validateattributes(verticalWeight,{'numeric'},{'scalar','finite','nonnegative'});
    startPoint=startPoint(:); goalPoint=goalPoint(:);
    waypoints=zeros(3,0);
    info=struct('status','no_path','capacity',0,'availableNodes',0, ...
        'edges',0,'buildSeconds',0,'searchSeconds',0, ...
        'startJoinClearance',startJoinClearance,'graphClearance',clearance, ...
        'verticalCostWeight',verticalWeight,'pathLength',nan, ...
        'ascent',nan,'descent',nan,'verticalTravel',nan,'pathCost',nan, ...
        'rawGraphCost',nan);
    lo=p.workspace(1,:)'; hi=p.workspace(2,:)';
    if ~buildOnly
    if any(startPoint<lo+startJoinClearance) || ...
            any(startPoint>hi-startJoinClearance) || ...
            any(goalPoint<lo+clearance) || any(goalPoint>hi-clearance)
        info.status='connector_out_of_bounds'; return;
    end
    if planner_segments_free3d(startPoint,goalPoint,obstacles,clearance)
        waypoints=goalPoint;
        info.status='direct';
        info.capacity=numel(lo(1):p.pathGridResolution:hi(1))* ...
            numel(lo(2):p.pathGridResolution:hi(2))*numel(p.heightLayers);
        info=add_route_metrics(info,[startPoint,waypoints],p);
        return;
    end
    end
    if isempty(cache)
    buildClock=tic;
    xs=lo(1):p.pathGridResolution:hi(1);
    ys=lo(2):p.pathGridResolution:hi(2);
    layers=p.heightLayers;
    [xx,yy,zz]=ndgrid(xs,ys,layers);
    dims=size(xx);
    info.capacity=numel(xx);
    freeClearance=inf(dims);
    for k=1:numel(obstacles)
        o=obstacles(k);
        if strcmp(o.type,'sphere')
            d=sqrt((xx-o.center(1)).^2+(yy-o.center(2)).^2+ ...
                (zz-o.center(3)).^2)-o.r;
        elseif strcmp(o.type,'cylinder')
            radial=sqrt((xx-o.center(1)).^2+(yy-o.center(2)).^2)-o.r;
            vertical=max(o.zmin-zz,zz-o.zmax);
            positiveR=max(radial,0); positiveZ=max(vertical,0);
            d=sqrt(positiveR.^2+positiveZ.^2)+min(max(radial,vertical),0);
        elseif strcmp(o.type,'box')
            qx=abs(xx-o.center(1))-o.halfSize(1);
            qy=abs(yy-o.center(2))-o.halfSize(2);
            qz=abs(zz-o.center(3))-o.halfSize(3);
            d=sqrt(max(qx,0).^2+max(qy,0).^2+max(qz,0).^2)+ ...
                min(max(max(qx,qy),qz),0);
        else
            error('UAV3D:ObstacleType','Unsupported obstacle type: %s',o.type);
        end
        freeClearance=min(freeClearance,d);
    end
    blocked=xx<lo(1)+clearance | xx>hi(1)-clearance | ...
        yy<lo(2)+clearance | yy>hi(2)-clearance | ...
        zz<lo(3)+clearance | zz>hi(3)-clearance | ...
        freeClearance<clearance;
    info.availableNodes=nnz(~blocked);
    coords=[xx(:)';yy(:)';zz(:)'];
    blockedFlat=blocked(:);
    clearFlat=freeClearance(:);
    dirs=[1 0 0;0 1 0;1 1 0;1 -1 0;0 0 1];
    src=zeros(0,1); dst=zeros(0,1); weights=zeros(0,1);
    [ix,iy,iz]=ind2sub(dims,(1:numel(xx))');
    for d=1:size(dirs,1)
        dr=dirs(d,1); dc=dirs(d,2); dl=dirs(d,3);
        valid=ix+dr>=1 & ix+dr<=dims(1) & ...
            iy+dc>=1 & iy+dc<=dims(2) & iz+dl<=dims(3);
        a=find(valid & ~blockedFlat);
        b=sub2ind(dims,ix(a)+dr,iy(a)+dc,iz(a)+dl);
        keep=~blockedFlat(b);
        a=a(keep); b=b(keep);
        if dr~=0 && dc~=0
            corner1=sub2ind(dims,ix(a)+dr,iy(a),iz(a));
            corner2=sub2ind(dims,ix(a),iy(a)+dc,iz(a));
            keep=~blockedFlat(corner1) & ~blockedFlat(corner2);
            a=a(keep); b=b(keep);
        end
        edgeLength=norm([dr;dc;0]*p.pathGridResolution+[0;0;dl*(layers(2)-layers(1))]);
        % The actual adjacent layer gap is used below (layers need not be uniform).
        if dl==1
            lengths=reshape(layers(iz(b))-layers(iz(a)),[],1);
        else
            lengths=edgeLength*ones(numel(a),1);
        end
        risky=find(min(clearFlat(a),clearFlat(b))<clearance+lengths/2);
        keep=true(numel(a),1);
        keep(risky)=planner_segments_free3d(coords(:,a(risky)),coords(:,b(risky)), ...
            obstacles,clearance-1e-9);
        src=[src;a(keep)]; %#ok<AGROW>
        dst=[dst;b(keep)]; %#ok<AGROW>
        dz=abs(coords(3,b(keep))-coords(3,a(keep)))';
        weights=[weights;lengths(keep)+verticalWeight*dz]; %#ok<AGROW>
    end
    info.edges=numel(src);
    info.buildSeconds=toc(buildClock);
    map=graph(src,dst,weights,numel(xx));
    cache=struct('map',map,'coords',coords,'blocked',blockedFlat, ...
        'capacity',info.capacity,'availableNodes',info.availableNodes, ...
        'edges',info.edges,'graphClearance',clearance, ...
        'verticalCostWeight',verticalWeight,'heightLayers',p.heightLayers, ...
        'workspace',p.workspace,'gridResolution',p.pathGridResolution, ...
        'obstacles',obstacles);
    else
        if cache.graphClearance~=clearance || ...
                cache.verticalCostWeight~=verticalWeight || ...
                ~isequal(cache.heightLayers,p.heightLayers) || ...
                ~isequal(cache.workspace,p.workspace) || ...
                cache.gridResolution~=p.pathGridResolution || ...
                ~isequaln(cache.obstacles,obstacles)
            error('UAV3D:PlannerCache','Planner cache configuration or map mismatch.');
        end
        map=cache.map;
        coords=cache.coords;
        blockedFlat=cache.blocked;
        info.capacity=cache.capacity;
        info.availableNodes=cache.availableNodes;
        info.edges=cache.edges;
    end
    if buildOnly || cache.edges==0, return; end
    startNode=nearest_visible(startPoint,coords,blockedFlat,obstacles,startJoinClearance);
    goalNode=nearest_visible(goalPoint,coords,blockedFlat,obstacles,clearance);
    if startNode==0 || goalNode==0
        info.status='connector_failed'; return;
    end
    searchClock=tic;
    [route,info.rawGraphCost]=shortestpath(map,startNode,goalNode);
    info.searchSeconds=toc(searchClock);
    if isempty(route), return; end
    raw=[startPoint,coords(:,route),goalPoint];
    changes=diff(raw,1,2);
    cumulativeCost=[0,cumsum(vecnorm(changes)+verticalWeight*abs(changes(3,:)))];
    keep=1;
    while keep(end)<size(raw,2)
        first=keep(end);
        next=first+1;
        required=clearance;
        if first==1, required=startJoinClearance; end
        eligible=zeros(1,0);
        for j=size(raw,2):-1:first+1
            delta=raw(:,j)-raw(:,first);
            shortcutCost=norm(delta)+verticalWeight*abs(delta(3));
            if shortcutCost<=cumulativeCost(j)-cumulativeCost(first)+1e-9
                eligible(end+1)=j; %#ok<AGROW>
            end
        end
        visible=planner_segments_free3d(repmat(raw(:,first),1,numel(eligible)), ...
            raw(:,eligible),obstacles,required-1e-9);
        chosen=find(visible,1);
        if ~isempty(chosen),next=eligible(chosen);end
        keep(end+1)=next; %#ok<AGROW>
    end
    waypoints=raw(:,keep(2:end));
    info.status='path';
    info=add_route_metrics(info,[startPoint,waypoints],p);
end

function info=add_route_metrics(info,points,p)
    m=route_cost3d(points,p);
    info.pathLength=m.pathLength; info.ascent=m.ascent; info.descent=m.descent;
    info.verticalTravel=m.verticalTravel; info.pathCost=m.cost;
end

function id=nearest_visible(point,coords,blocked,obstacles,clearance)
    candidate=find(~blocked);
    distance=sum((coords(:,candidate)-point).^2,1);
    [~,order]=sort(distance);
    id=0;
    selected=candidate(order(1:min(200,numel(order))));
    visible=planner_segments_free3d(repmat(point,1,numel(selected)), ...
        coords(:,selected),obstacles,clearance-1e-9);
    first=find(visible,1);
    if ~isempty(first),id=selected(first);end
end

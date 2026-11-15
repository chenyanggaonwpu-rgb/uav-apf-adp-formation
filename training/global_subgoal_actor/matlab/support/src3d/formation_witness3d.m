function witness=formation_witness3d(startPoint,goalPoint,offsets,obstacles,p,rules)
% FORMATION_WITNESS3D Independent full-3-D translation search, with continuous certificates.
% Six-neighbour XYZ search does not call the evaluated layer planner or controller.
% A fixed-heading V is a sufficient witness, not a necessary feasibility condition.
    witness=struct('status','unknown','method','independent_xyz_translation_v1', ...
        'points',zeros(3,0),'offsets',offsets,'expandedNodes',0, ...
        'clearanceLowerBound',nan,'minPairDistance',nan,'duration',nan, ...
        'segmentDurations',zeros(1,0),'segmentPeakSpeeds',zeros(1,0), ...
        'maxAcceleration',p.amax,'settleDuration',p.terminalBlendTime+p.holdDuration, ...
        'pathLength',nan,'verticalTravel',nan);
    pair=inf;
    for i=1:size(offsets,2)-1
        for j=i+1:size(offsets,2), pair=min(pair,norm(offsets(:,i)-offsets(:,j))); end
    end
    witness.minPairDistance=pair;
    if pair<=p.uavSafe, witness.status='invalid_pair_geometry'; return; end
    [direct,~]=certify(startPoint,goalPoint);
    if direct
        points=[startPoint,goalPoint];
    else
        lo=p.workspace(1,:)'; hi=p.workspace(2,:)'; step=rules.witnessGridResolution;
        xs=unique([lo(1):step:hi(1),startPoint(1),goalPoint(1)]);
        ys=unique([lo(2):step:hi(2),startPoint(2),goalPoint(2)]);
        zs=unique([lo(3):step:hi(3),startPoint(3),goalPoint(3)]);
        [xx,yy,zz]=ndgrid(xs,ys,zs); dims=[numel(xs),numel(ys),numel(zs)];
        coords=[xx(:)';yy(:)';zz(:)']; count=size(coords,2);
        gaps=inf(1,count); blocked=false(1,count);
        for vehicle=1:size(offsets,2)
            shifted=coords+offsets(:,vehicle);
            gaps=min(gaps,point_clearance_batch3d(shifted,obstacles));
            blocked=blocked | any(shifted<lo+rules.boundaryClearance | ...
                shifted>hi-rules.boundaryClearance,1);
        end
        blocked=blocked | gaps<rules.witnessClearance;
        first=sub2ind(dims,find(xs==startPoint(1),1),find(ys==startPoint(2),1),find(zs==startPoint(3),1));
        last=sub2ind(dims,find(xs==goalPoint(1),1),find(ys==goalPoint(2),1),find(zs==goalPoint(3),1));
        if blocked(first) || blocked(last), witness.status='endpoint_not_certified'; return; end
        open=false(1,count); closed=open; open(first)=true;
        costs=inf(1,count); costs(first)=0; parents=zeros(1,count);
        heuristic=vecnorm(coords-goalPoint); found=false;
        directions=[1 0 0;-1 0 0;0 1 0;0 -1 0;0 0 1;0 0 -1];
        while any(open) && witness.expandedNodes<rules.maxExpandedNodes
            ids=find(open); [~,position]=min(costs(ids)+heuristic(ids)); current=ids(position);
            open(current)=false; closed(current)=true; witness.expandedNodes=witness.expandedNodes+1;
            if current==last, found=true; break; end
            [ix,iy,iz]=ind2sub(dims,current); indices=[ix iy iz];
            for k=1:6
                nextIndex=indices+directions(k,:);
                if any(nextIndex<1) || any(nextIndex>dims), continue; end
                next=sub2ind(dims,nextIndex(1),nextIndex(2),nextIndex(3));
                if blocked(next) || closed(next), continue; end
                distance=norm(coords(:,next)-coords(:,current)); trial=costs(current)+distance;
                if trial>=costs(next), continue; end
                [valid,~]=certify_formation_segment3d(coords(:,current),coords(:,next), ...
                    offsets,obstacles,p,rules.witnessClearance,rules.boundaryClearance, ...
                    rules.certificateTolerance,gaps(current),gaps(next));
                if ~valid, continue; end
                costs(next)=trial; parents(next)=current; open(next)=true;
            end
        end
        if ~found
            if any(open), witness.status='search_budget_exhausted';
            else, witness.status='no_grid_witness'; end
            return;
        end
        route=last;
        while route(1)~=first, route=[parents(route(1)),route]; end %#ok<AGROW>
        raw=coords(:,route); points=raw(:,1); cursor=1;
        while cursor<size(raw,2)
            for next=size(raw,2):-1:cursor+1
                [valid,~]=certify(raw(:,cursor),raw(:,next));
                if valid, break; end
            end
            points(:,end+1)=raw(:,next); cursor=next; %#ok<AGROW>
        end
    end
    % Preserve the mandated departure before the independently searched path.
    if norm(points(:,1)-p.start)>1e-12, points=[p.start,points]; end
    if norm(points(:,end)-p.target)>1e-12, points=[points,p.target]; end
    differences=diff(points,1,2); lengths=vecnorm(differences);
    durations=zeros(size(lengths)); peaks=zeros(size(lengths)); lower=inf;
    for k=1:numel(lengths)
        [valid,bound]=certify(points(:,k),points(:,k+1));
        if ~valid, witness.status='segment_not_certified'; return; end
        lower=min(lower,bound); length=lengths(k);
        peaks(k)=min(p.vmax,sqrt(p.amax*length));
        durations(k)=length/max(peaks(k),eps)+peaks(k)/p.amax;
    end
    witness.points=points; witness.clearanceLowerBound=lower;
    witness.segmentDurations=durations; witness.segmentPeakSpeeds=peaks;
    witness.duration=sum(durations)+witness.settleDuration;
    witness.pathLength=sum(lengths); witness.verticalTravel=sum(abs(differences(3,:)));
    if witness.duration>p.Tmax, witness.status='time_budget_not_certified';
    else, witness.status='certified'; end
    function [ok,bound]=certify(a,b)
        [ok,bound]=certify_formation_segment3d(a,b,offsets,obstacles,p, ...
            rules.witnessClearance,rules.boundaryClearance,rules.certificateTolerance);
    end
end

function [point,info]=rl_local_hop3d(action,position,previous,obstacles,p,settings)
% 3D local reachability executor (final step-0 version).
% Leader path must be clear at >= leader_clearance; five-body clearance is
% preferred but not required (formation may compress through a window).
    raw=double(action(:));
    assert(numel(raw)==3&&all(isfinite(raw)));
    lo=p.workspace(1,:)';hi=p.workspace(2,:)';
    requested=lo+(max(-1,min(1,raw))+1).*.5.*(hi-lo);
    if isfield(settings,'request_mode')&&strcmp(settings.request_mode,'goal')
        % Control arm: replace the actor's destination by "50 m toward the goal".
        d=double(p.target(1:2))-position(1:2);
        if norm(d)<1e-6,d=[1;0];end
        requested=[position(1:2)+50*d/norm(d);position(3)];
        requested(1)=max(lo(1),min(hi(1),requested(1)));
        requested(2)=max(lo(2),min(hi(2),requested(2)));
    end
    clearance=max(settings.projection_clearance,p.obsSafe+.05);
    leaderClear=max(field_or(settings,'leader_clearance',1.5),clearance);
    window=field_or(settings,'local_window',24);
    lateral=field_or(settings,'local_grid',2);
    vert=field_or(settings,'local_zgrid',3);
    hopMax=field_or(settings,'local_hop_max',12);
    offsets=field_or(settings,'formation_offsets',zeros(3,5));
    method='requested_ok';
    point=requested;
    if ~segment_ok(position,requested,obstacles,clearance,leaderClear,offsets)
        [point,method]=hop3d(position,requested,obstacles,clearance,leaderClear, ...
            window,lateral,vert,hopMax,offsets,lo,hi);
    end
    if norm(point-position)<.5
        direction=requested-position;
        if norm(direction)<1e-8,direction=raw;end
        if norm(direction)<1e-8,direction=[1;0;0];end
        point=position+.5*direction/norm(direction);
        method=[method '_degenerate'];
    end
    point=max(lo,min(hi,point));
    repeated=all(isfinite(previous))&&norm(point-previous)<.5;
    info=struct('raw_action',raw,'requested',requested,'executed',point, ...
        'projection_distance',norm(point-requested),'method',method, ...
        'near_requested',false,'repeated',repeated,'hop_length',norm(point-position));
end

function [point,method]=hop3d(position,requested,obstacles,clearance,leaderClear, ...
        window,lateral,vert,hopMax,offsets,lo,hi)
    gx=position(1)+(-window:lateral:window);
    gy=position(2)+(-window:lateral:window);
    zlo=max(lo(3)+1,position(3)-9);
    zhi=min(hi(3)-1,position(3)+9);
    gz=unique([zlo:vert:zhi,position(3)]);
    [GX,GY,GZ]=ndgrid(gx,gy,gz);
    pts=[GX(:)';GY(:)';GZ(:)'];
    free=reshape(point_clearance_batch3d(pts,obstacles)>=leaderClear,size(GX));
    inb=pts(1,:)>=lo(1)+.5&pts(1,:)<=hi(1)-.5&pts(2,:)>=lo(2)+.5&pts(2,:)<=hi(2)-.5;
    free=free&reshape(inb,size(GX));
    [~,ix]=min(abs(gx-position(1)));[~,iy]=min(abs(gy-position(2)));
    [~,iz]=min(abs(gz-position(3)));
    if ~free(iy,ix,iz)
        point=requested;method='no_free_start';return;
    end
    [seen,parent,depth]=bfs3(free,iy,ix,iz);
    [ii,jj]=ndgrid(1:numel(gy),1:numel(gx));
    dx=reshape(gx(jj)-position(1),size(seen,1),size(seen,2),1);
    dy=reshape(gy(ii)-position(2),size(seen,1),size(seen,2),1);
    dz=GZ-position(3);
    rx=requested(1)-position(1);ry=requested(2)-position(2);rz=requested(3)-position(3);
    rn=norm([rx ry rz]);if rn<1e-6,rn=1;end
    proj=dx*rx+dy*ry+dz*rz;
    onEdge=max(abs(dx),abs(dy))>=window-lateral;
    forward=seen&onEdge&proj>0.15*rn*window;
    if any(forward(:))
        candidates=forward;
    else
        candidates=seen&onEdge;   % pocket: no forward exit inside the window
    end
    if ~any(candidates(:))
        point=requested;method='no_window_exit';return;
    end
    score=proj/rn-0.5*depth*lateral/10;
    score(~candidates)=-inf;
    [~,linear]=max(score(:));
    [pi,pj,pk]=ind2sub(size(seen),linear);
    path=zeros(0,3);c=sub2ind(size(seen),pi,pj,pk);
    while c>0
        [a,b,d]=ind2sub(size(seen),c);
        path=[a b d;path]; %#ok<AGROW>
        if a==iy&&b==ix&&d==iz,break;end
        c=parent(a,b,d);
    end
    % Prefer five-body verified points; fall back to leader-verified only.
    point5=[];hop5=0;point1=[];hop1=0;
    for r=1:size(path,1)
        q=[gx(path(r,2));gy(path(r,1));gz(path(r,3))];
        d=norm(q-position);
        if d<1.0||d>hopMax,continue;end
        if segment_ok(position,q,obstacles,clearance,leaderClear,offsets)
            if d>hop5,point5=q;hop5=d;end
        elseif leader_segment_ok(position,q,obstacles,leaderClear)
            if d>hop1,point1=q;hop1=d;end
        end
    end
    if ~isempty(point5)
        point=point5;method='local_hop3d';
    elseif ~isempty(point1)
        point=point1;method='local_hop3d_leaderonly';
    else
        % Never fall back to the raw exit: it can be far behind a wall.
        for r=size(path,1):-1:1
            q=[gx(path(r,2));gy(path(r,1));gz(path(r,3))];
            if norm(q-position)<=hopMax,point=q;break;end
        end
        if isempty(point),point=[gx(path(1,2));gy(path(1,1));gz(path(1,3))];end
        method='local_hop3d_unverified';
    end
end

function [seen,parent,depth]=bfs3(free,si,sj,sk)
    seen=false(size(free));parent=zeros(size(free));depth=inf(size(free));
    seen(si,sj,sk)=true;depth(si,sj,sk)=0;
    queue=zeros(numel(free),1);queue(1)=sub2ind(size(free),si,sj,sk);
    head=1;tail=1;
    while head<=tail
        c=queue(head);head=head+1;
        [ci,cj,ck]=ind2sub(size(free),c);
        for d=[1 0 0;-1 0 0;0 1 0;0 -1 0;0 0 1;0 0 -1]'
            ni=ci+d(1);nj=cj+d(2);nk=ck+d(3);
            if ni>=1&&ni<=size(free,1)&&nj>=1&&nj<=size(free,2)&&nk>=1&&nk<=size(free,3)&& ...
                    free(ni,nj,nk)&&~seen(ni,nj,nk)
                seen(ni,nj,nk)=true;parent(ni,nj,nk)=c;
                depth(ni,nj,nk)=depth(ci,cj,ck)+1;
                tail=tail+1;queue(tail)=sub2ind(size(free),ni,nj,nk);
            end
        end
    end
end

function ok=segment_ok(a,b,obstacles,clearance,leaderClear,offsets)
% Five-body straight segment, then the leader-only margin as a hard floor.
    ok=leader_segment_ok(a,b,obstacles,leaderClear);
    if ~ok,return;end
    n=max(2,ceil(norm(b-a)/.5));n=double(n);s=linspace(0,1,n);
    pts=a+(b-a)*s;
    for k=1:size(offsets,2)
        if any(point_clearance_batch3d(pts+offsets(:,k),obstacles)<clearance)
            ok=false;return;
        end
    end
end

function ok=leader_segment_ok(a,b,obstacles,leaderClear)
    a=a(:);b=b(:);n=max(2,ceil(norm(b-a)/.5));n=double(n);
    pts=a+(b-a)*linspace(0,1,n);
    ok=all(point_clearance_batch3d(pts,obstacles)>=leaderClear);
end

function v=field_or(s,name,default)
    if isfield(s,name)&&~isempty(s.(name)),v=s.(name);else,v=default;end
end

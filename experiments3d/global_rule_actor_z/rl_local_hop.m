function [point,info]=rl_local_hop(action,position,previous,obstacles,p,settings)
% Local reachability executor (diagnostic, "step 0").
% The actor still chooses the destination; this function only refuses to
% execute a first step that cannot be flown straight from here.
% Selection uses reachability, acceptance uses the five-body segment check.
    raw=double(action(:));
    assert(numel(raw)==3&&all(isfinite(raw)));
    if isfield(settings,'hop_mode')&&strcmp(settings.hop_mode,'3d')
        [point,info]=rl_local_hop3d(action,position,previous,obstacles,p,settings);
        return;
    end
    lo=p.workspace(1,:)';hi=p.workspace(2,:)';
    requested=lo+(max(-1,min(1,raw))+1).*.5.*(hi-lo);
    if isfield(settings,'request_mode') && any(strcmp(settings.request_mode,{'goal','goal_xy'}))
        % Use the existing goal direction. goal_xy retains the actor's altitude.
        d=double(p.target(1:2))-position(1:2);
        if norm(d)<1e-6,d=[1;0];end
        requested(1:2)=max(lo(1:2),min(hi(1:2),position(1:2)+50*d/norm(d)));
        if strcmp(settings.request_mode,'goal')
            requested(3)=position(3);
        end
    end
    clearance=max(settings.projection_clearance,p.obsSafe+.05);
    window=field_or(settings,'local_window',24);
    grid=field_or(settings,'local_grid',1);
    hopMax=field_or(settings,'local_hop_max',12);
    offsets=field_or(settings,'formation_offsets',zeros(3,5));
    method='requested_ok';hop=norm(requested-position);bypass=false;
    latched=isfield(settings,'bypass_until')&&isfield(settings,'now')&& ...
        settings.now<settings.bypass_until;
    if latched
        point=requested;method='window_bypass_hold';bypass=true;
    elseif ~segment_ok(position,requested,obstacles,offsets,clearance)
        if field_or(settings,'gate_windows',true)&& ...
                window_bypass(position,requested,obstacles,p,settings)
            % The only way forward needs a climb/dive: let the actor's
            % altitude request drive, do not route it in the flat plane.
            point=requested;method='window_bypass';bypass=true;hop=norm(requested-position);
        else
            [point,method,hop]=hop_from_window(position,requested,obstacles, ...
                clearance,window,grid,hopMax,offsets);
        end
    else
        point=requested;
    end
    if norm(point-position)<.5
        % Never hand back a point that is already here; that stalls the clock.
        direction=requested-position;
        if norm(direction)<1e-8,direction=raw;end
        if norm(direction)<1e-8,direction=[1;0;0];end
        point=position+.5*direction/norm(direction);
        method=[method '_degenerate'];
    end
    repeated=all(isfinite(previous))&&norm(point-previous)<.5;
    info=struct('raw_action',raw,'requested',requested,'executed',point, ...
        'projection_distance',norm(point-requested),'method',method, ...
        'near_requested',false,'repeated',repeated,'hop_length',norm(point-position),'bypass',bypass);
end

function [point,method,hop]=hop_from_window(position,requested,obstacles, ...
        clearance,window,grid,hopMax,offsets)
% Reachable set inside the window, then the boundary exit closest in angle
% to the requested direction; the hop is the furthest flyable point on that
% local path.
    gx=position(1)+(-window:grid:window);
    gy=position(2)+(-window:grid:window);
    [GX,GY]=meshgrid(gx,gy);
    GZ=position(3)*ones(size(GX));
    pts=[GX(:)';GY(:)';GZ(:)'];
    free=point_clearance_batch3d(pts,obstacles)>=clearance;
    free=reshape(free,numel(gy),numel(gx));
    [~,sj]=min(abs(gx-position(1)));[~,si]=min(abs(gy-position(2)));
    if ~free(si,sj)
        point=requested;method='no_free_start';hop=norm(requested-position);return;
    end
    [seen,parent,depth]=bfs_grid(free,si,sj);
    [ii,jj]=find(seen);
    dx=gx(jj)'-position(1);dy=gy(ii)'-position(2);
    radius=hypot(dx,dy);
    onEdge=max(abs(dx),abs(dy))>=window-1.5*grid;
    dirRequest=requested(1:2)-position(1:2);
    if norm(dirRequest)<1e-6,dirRequest=[1;0];end
    dirRequest=dirRequest/norm(dirRequest);
    score=(dx*dirRequest(1)+dy*dirRequest(2))./max(radius,1e-9);
    score(~onEdge)=-inf;
    if ~any(isfinite(score))
        % No boundary exit: take the reachable cell furthest along the request.
        [~,k]=max(score);
        point=requested;
        method='no_window_exit';hop=norm(requested-position);
        return;
    end
    [~,k]=max(score);
    ei=ii(k);ej=jj(k);
    % Backtrack the local path.
    path=[];
    c=sub2ind(size(free),ei,ej);
    while c>0
        [pi,pj]=ind2sub(size(free),c);
        path=[pi pj;path]; %#ok<AGROW>
        if pi==si&&pj==sj,break;end
        c=parent(pi,pj);
    end
    % Walk forward, keep the furthest point that is still flyable.
    point=[];hop=0;method='local_hop';
    for r=1:size(path,1)
        q=[gx(path(r,2));gy(path(r,1));position(3)];
        d=norm(q-position);
        if d<1.0,continue;end
        if d>hopMax,continue;end
        if segment_ok(position,q,obstacles,offsets,clearance)
            if d>hop,point=q;hop=d;end
        end
    end
    if isempty(point)
        q=[gx(path(end,2));gy(path(end,1));position(3)];
        point=q;hop=norm(q-position);method='local_hop_unverified';
    end
end

function [seen,parent,depth]=bfs_grid(free,si,sj)
    seen=false(size(free));parent=zeros(size(free));depth=inf(size(free));
    seen(si,sj)=true;depth(si,sj)=0;
    queue=zeros(numel(free),1);queue(1)=sub2ind(size(free),si,sj);
    head=1;tail=1;
    while head<=tail
        c=queue(head);head=head+1;
        [ci,cj]=ind2sub(size(free),c);
        for d=[1 0;-1 0;0 1;0 -1]'
            ni=ci+d(1);nj=cj+d(2);
            if ni>=1&&ni<=size(free,1)&&nj>=1&&nj<=size(free,2)&& ...
                    free(ni,nj)&&~seen(ni,nj)
                seen(ni,nj)=true;parent(ni,nj)=c;
                depth(ni,nj)=depth(ci,cj)+1;
                tail=tail+1;queue(tail)=sub2ind(size(free),ni,nj);
            end
        end
    end
end

function ok=segment_ok(a,b,obstacles,offsets,clearance)
% Five-body straight segment: every offset body keeps the clearance.
    a=a(:);b=b(:);n=max(2,ceil(norm(b-a)/.5));n=double(n);
    s=linspace(0,1,n);
    pts=a+(b-a)*s;
    ok=true;
    for k=1:size(offsets,2)
        q=pts+offsets(:,k);
        if any(point_clearance_batch3d(q,obstacles)<clearance)
            ok=false;return;
        end
    end
end

function v=field_or(s,name,default)
    if isfield(s,name)&&~isempty(s.(name)),v=s.(name);else,v=default;end
end

function tf=window_bypass(position,requested,obstacles,p,settings)
% True when the flat reachable set has no forward exit but a 3D exit does:
% a window/door that only the altitude request can use.
    window=field_or(settings,'local_window',24);
    cosMin=field_or(settings,'gate_cos',0.7);
    lo=p.workspace(1,:)';hi=p.workspace(2,:)';
    clr=max(settings.projection_clearance,p.obsSafe+.05);
    rx=requested(1)-position(1);ry=requested(2)-position(2);
    rn=max(norm([rx ry]),1e-6);
    % flat layer at the leader altitude
    gx=position(1)+(-window:1:window);
    gy=position(2)+(-window:1:window);
    [X,Y]=meshgrid(gx,gy);
    P=[X(:)';Y(:)';position(3)*ones(1,numel(X))];
    F=reshape(point_clearance_batch3d(P,obstacles)>=clr,size(X));
    inb=P(1,:)>=lo(1)+.5&P(1,:)<=hi(1)-.5&P(2,:)>=lo(2)+.5&P(2,:)<=hi(2)-.5;
    F=F&reshape(inb,size(X));
    if forward_2d(F,gx,gy,position,rx,ry,rn,window,cosMin)
        tf=false;return;
    end
    % 3D layers
    gz=unique([max(lo(3)+1,position(3)-12):4:min(hi(3)-1,position(3)+12),position(3)]);
    [GX,GY,GZ]=ndgrid(gx(1:3:end),gy(1:3:end),gz);
    P3=[GX(:)';GY(:)';GZ(:)'];
    F3=reshape(point_clearance_batch3d(P3,obstacles)>=clr,size(GX));
    inb3=P3(1,:)>=lo(1)+.5&P3(1,:)<=hi(1)-.5&P3(2,:)>=lo(2)+.5&P3(2,:)<=hi(2)-.5;
    F3=F3&reshape(inb3,size(GX));
    [~,ix]=min(abs(gx(1:3:end)-position(1)));[~,iy]=min(abs(gy(1:3:end)-position(2)));
    [~,iz]=min(abs(gz-position(3)));
    if ~F3(iy,ix,iz),tf=true;return;end
    S3=flood3(F3,iy,ix,iz);
    gx3=gx(1:3:end);gy3=gy(1:3:end);
    [ii,jj]=ndgrid(1:numel(gy3),1:numel(gx3));
    dx=reshape(gx3(jj)-position(1),size(S3,1),size(S3,2),1);
    dy=reshape(gy3(ii)-position(2),size(S3,1),size(S3,2),1);
    proj=dx*rx+dy*ry;
    edge=max(abs(dx),abs(dy))>=window-3;
    cosang=proj./(rn.*max(hypot(dx,dy),1e-6));
    tf=any(any(any(S3&edge&cosang>cosMin)));
end

function tf=forward_2d(F,gx,gy,position,rx,ry,rn,window,cosMin)
    [ii,jj]=ndgrid(1:numel(gy),1:numel(gx));
    dx=gx(jj)-position(1);dy=gy(ii)-position(2);
    [si,sj]=find(F,1);tf=false;
    if isempty(si),return;end
    [~,sx]=min(abs(gx-position(1)));[~,sy]=min(abs(gy-position(2)));
    if ~F(sy,sx),return;end
    S=flood2(F,sy,sx);
    proj=dx*rx+dy*ry;
    edge=max(abs(dx),abs(dy))>=window-1.5;
    cosang=proj./(rn.*max(hypot(dx,dy),1e-6));
    tf=any(any(S&edge&cosang>cosMin));
end

function S=flood2(F,si,sj)
    S=false(size(F));S(si,sj)=true;
    q=zeros(numel(F),1);q(1)=sub2ind(size(F),si,sj);h=1;t=1;
    while h<=t
        c=q(h);h=h+1;[ci,cj]=ind2sub(size(F),c);
        for d=[1 0;-1 0;0 1;0 -1]'
            ni=ci+d(1);nj=cj+d(2);
            if ni>=1&&ni<=size(F,1)&&nj>=1&&nj<=size(F,2)&&F(ni,nj)&&~S(ni,nj)
                S(ni,nj)=true;t=t+1;q(t)=sub2ind(size(F),ni,nj);
            end
        end
    end
end

function S=flood3(F,si,sj,sk)
    S=false(size(F));S(si,sj,sk)=true;
    q=zeros(numel(F),1);q(1)=sub2ind(size(F),si,sj,sk);h=1;t=1;
    while h<=t
        c=q(h);h=h+1;[ci,cj,ck]=ind2sub(size(F),c);
        for d=[1 0 0;-1 0 0;0 1 0;0 -1 0;0 0 1;0 0 -1]'
            ni=ci+d(1);nj=cj+d(2);nk=ck+d(3);
            if ni>=1&&ni<=size(F,1)&&nj>=1&&nj<=size(F,2)&&nk>=1&&nk<=size(F,3)&&F(ni,nj,nk)&&~S(ni,nj,nk)
                S(ni,nj,nk)=true;t=t+1;q(t)=sub2ind(size(F),ni,nj,nk);
            end
        end
    end
end





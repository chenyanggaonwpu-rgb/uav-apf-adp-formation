function [refs,vrefs,psi,blocked] = formation_reference3d(state,memory,obstacles,p)
% FORMATION_REFERENCE3D Horizontal arc-length lag plus synchronized height.
% The reference is not a fixed 3-D arc-length or rigid-body V trajectory.
    lead=state(1:3,1); leadVel=state(4:6,1);
    horizontalSpeed=norm(leadVel(1:2));
    psi=memory.psi;
    if horizontalSpeed>0.05 && ~memory.hold
        psi=atan2(leadVel(2),leadVel(1));
    end
    if memory.hold, psi=memory.holdPsi; end
    rot=[cos(psi) -sin(psi);sin(psi) cos(psi)];
    refs=zeros(3,p.nUAV);vrefs=zeros(3,p.nUAV);
    directional=isfield(p,'directionalClearance') && p.directionalClearance.enable;
    footprints=[];
    if directional
        footprints=reference_footprints3d(obstacles,lead(3), ...
            p.desiredClear,p.directionalClearance.referenceZ);
    end
    refs(:,1)=lead;vrefs(:,1)=leadVel;
    for i=2:p.nUAV
        row=ceil((i-1)/2);
        side=2*mod(i,2)-1;
        if memory.hold
            fixed=[lead(1:2)+rot*memory.offsets(:,i);lead(3)];
            refs(:,i)=(1-memory.holdAlpha)*memory.holdStartRefs(:,i)+ ...
                memory.holdAlpha*fixed;
            vrefs(:,i)=leadVel;
        else
            lag=row*p.formSpacing;
            [trailPoint,dir,travel]=trail_point_xy(memory.trail,memory.cum, ...
                memory.count,lag,psi,p.noiseStd);
            lateral=side*row*p.formWing;
            refs(:,i)=[trailPoint+lateral*[-dir(2);dir(1)]- ...
                max(0,lag-travel)*dir;lead(3)];
            vrefs(:,i)=[horizontalSpeed*dir;leadVel(3)];
        end
        refs(:,i)=clear_reference_xy(refs(:,i),obstacles,p,psi,footprints,directional);
    end
    % Keep independently projected references from collapsing onto each other.
    for pass=1:5
        changed=false;
        for i=1:p.nUAV-1
            for j=i+1:p.nUAV
                diff=refs(1:2,j)-refs(1:2,i);
                distance=norm(refs(:,j)-refs(:,i));
                if distance<p.uavSafe+0.2
                    if norm(diff)<1e-9, diff=[-sin(psi);cos(psi)]; end
                    direction=diff/norm(diff);
                    shift=0.5*(p.uavSafe+0.2-distance)*direction;
                    if i==1
                        refs(1:2,j)=refs(1:2,j)+2*shift;
                    else
                        refs(1:2,i)=refs(1:2,i)-shift;
                        refs(1:2,j)=refs(1:2,j)+shift;
                    end
                    changed=true;
                end
            end
        end
        if ~changed, break; end
        for i=2:p.nUAV
            refs(:,i)=clear_reference_xy(refs(:,i),obstacles,p,psi,footprints,directional);
        end
    end
    blocked=false;
    for i=2:p.nUAV
        if directional
            % The same precomputed height slice also certifies references;
            % avoid repeated ellipsoid projections for four synchronized UAVs.
            tooClose=false;
            for f=footprints
                delta=refs(1:2,i)-f.center;
                if strcmp(f.type,'circle')
                    gap=norm(delta)-f.padding;
                else
                    qxy=abs(delta)-f.half;
                    gap=norm(max(qxy,0))+min(max(qxy),0)-f.padding;
                end
                if gap < -1e-6*p.desiredClear, tooClose=true; break; end
            end
        else
            tooClose=segment_clearance3d(refs(:,i),refs(:,i),obstacles)<p.desiredClear-1e-6;
        end
        if tooClose
            blocked=true;
        end
    end
    for i=1:p.nUAV-1
        for j=i+1:p.nUAV
            if norm(refs(:,i)-refs(:,j))<p.uavSafe+0.05
                blocked=true;
            end
        end
    end
end

function [point,dir,total]=trail_point_xy(trail,cum,count,lag,psi,noiseStd)
    total=cum(count);
    target=max(0,total-lag);
    left=1;right=count;
    while left<right
        mid=floor((left+right)/2);
        if cum(mid)>=target, right=mid; else, left=mid+1; end
    end
    idx=left;
    if idx==1
        point=trail(:,1);
    else
        fraction=(target-cum(idx-1))/max(cum(idx)-cum(idx-1),eps);
        point=trail(:,idx-1)+fraction*(trail(:,idx)-trail(:,idx-1));
    end
    a=max(1,idx-1);b=min(count,idx+1);
    delta=trail(:,b)-trail(:,a);
    if norm(delta)>max(1e-6,3*noiseStd)
        dir=delta/norm(delta);
    else
        dir=[cos(psi);sin(psi)];
    end
end

function ref=clear_reference_xy(ref,obstacles,p,psi,footprints,directional)
    lo=p.workspace(1,1:2)'+p.obsSafe;
    hi=p.workspace(2,1:2)'-p.obsSafe;
    if directional
        for pass=1:10
            moved=false;
            for f=footprints
                delta=ref(1:2)-f.center;
                if strcmp(f.type,'box')
                    nearest=max(-f.half,min(f.half,delta));
                    outside=delta-nearest;
                    if norm(outside)>1e-12
                        if norm(outside)<f.padding
                            ref(1:2)=f.center+nearest+f.padding*outside/norm(outside);
                            moved=true;
                        end
                    else
                        [~,face]=min([f.half(1)-delta(1),f.half(1)+delta(1), ...
                            f.half(2)-delta(2),f.half(2)+delta(2)]);
                        axisIndex=ceil(face/2); direction=1-2*mod(face+1,2);
                        ref(axisIndex)=f.center(axisIndex)+direction*(f.half(axisIndex)+f.padding);
                        moved=true;
                    end
                else
                    d=norm(delta);
                    if d<f.padding
                        if d<1e-9, delta=[-sin(psi);cos(psi)]; d=1; end
                        ref(1:2)=f.center+f.padding*delta/d; moved=true;
                    end
                end
            end
            ref(1:2)=max(lo,min(hi,ref(1:2)));
            if ~moved, break; end
        end
        return;
    end
    for pass=1:10
        moved=false;
        for k=1:numel(obstacles)
            o=obstacles(k);
            if strcmp(o.type,'sphere')
                vertical=abs(ref(3)-o.center(3));
                radius=o.r+p.desiredClear;
                if vertical>=radius, continue; end
                required=sqrt(radius^2-vertical^2);
                center=o.center(1:2);
            elseif strcmp(o.type,'box')
                vertical=max([o.zmin-ref(3),ref(3)-o.zmax,0]);
                if vertical>=p.desiredClear, continue; end
                required=sqrt(p.desiredClear^2-vertical^2);
                center=o.center(1:2); half=o.halfSize(1:2);
                delta=ref(1:2)-center;
                nearest=max(-half,min(half,delta));
                outside=delta-nearest;
                if norm(outside)>1e-12
                    if norm(outside)<required
                        ref(1:2)=center+nearest+required*outside/norm(outside);
                        moved=true;
                    end
                else
                    [~,face]=min([half(1)-delta(1),half(1)+delta(1), ...
                        half(2)-delta(2),half(2)+delta(2)]);
                    axisIndex=ceil(face/2); direction=1-2*mod(face+1,2);
                    ref(axisIndex)=center(axisIndex)+direction*(half(axisIndex)+required);
                    moved=true;
                end
                continue;
            elseif strcmp(o.type,'cylinder')
                vertical=max([o.zmin-ref(3),ref(3)-o.zmax,0]);
                if vertical>=p.desiredClear, continue; end
                required=o.r+sqrt(p.desiredClear^2-vertical^2);
                center=o.center(1:2);
            else
                error('UAV3D:ObstacleType','Unsupported obstacle type: %s',o.type);
            end
            delta=ref(1:2)-center;
            d=norm(delta);
            if d<required
                if d<1e-9, delta=[-sin(psi);cos(psi)]; d=1; end
                ref(1:2)=center+required*delta/d;
                moved=true;
            end
        end
        ref(1:2)=max(lo,min(hi,ref(1:2)));
        if ~moved, break; end
    end
end

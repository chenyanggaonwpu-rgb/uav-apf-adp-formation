function [plan,safe]=f5_braking_plan(state,obstacles,p,s)
% Joint maximum-deceleration backup for five synchronized original plants.
% Individual stopping paths are collinear, so one complete segment certificate
% covers every individual brake step. Pair paths require every time interval.
    limit=ceil(p.vmax/(p.amax*p.dt))+3;
    states=zeros(6,p.nUAV,limit+1);states(:,:,1)=state;
    commands=zeros(3,p.nUAV,limit);count=0;safe=true;margin=inf;
    for k=1:limit
        command=limit_vector3d(-state(4:6,:)/p.dt,p.amax);
        next=motion_step3d(state,command,zeros(3,p.nUAV),p);
        commands(:,:,k)=command;states(:,:,k+1)=next;count=k;state=next;
        if max(vecnorm(state(4:6,:)))<=s.stopSpeed,break;end
    end
    if max(vecnorm(state(4:6,:)))>s.stopSpeed,safe=false;end
    [ok,margin]=f5_static_cert(states(1:3,:,1),states(1:3,:,count+1),obstacles,p,s);
    safe=safe && ok;
    if safe
        for i=1:p.nUAV-1
            for j=i+1:p.nUAV
                relative=reshape(states(1:3,i,1:count+1)-states(1:3,j,1:count+1),3,[]);
                a=relative(:,1:end-1);dr=diff(relative,1,2);
                alpha=max(0,min(1,-sum(a.*dr,1)./max(sum(dr.^2,1),eps)));
                m=min(vecnorm(a+dr.*alpha))-p.uavSafe;margin=min(margin,m);
                if m<=s.tolerance,safe=false;break;end
            end
            if ~safe,break;end
        end
    end
    plan=struct('states',states(:,:,1:count+1),'commands',commands(:,:,1:count), ...
        'margin',margin,'certified',safe);
end

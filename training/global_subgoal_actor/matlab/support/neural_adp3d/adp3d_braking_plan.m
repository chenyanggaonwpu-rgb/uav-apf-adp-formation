function [plan,safe] = adp3d_braking_plan(state,obstacles,p,settings)
% 按原 motion_step3d 半隐式离散动力学构造最大减速度刹停后备。
    plan=struct('states',state,'commands',zeros(3,0),'margin',inf,'certified',false);
    safe=true;
    limit=ceil(p.vmax/(p.amax*p.dt))+3;
    for k=1:limit
        command=-state(4:6)/p.dt;
        next=motion_step3d(state,command,zeros(3,1),p);
        [ok,margin]=adp3d_certify_segment(state(1:3),next(1:3),obstacles,p,settings);
        if ~ok, safe=false; return; end
        plan.commands(:,end+1)=command;
        plan.states(:,end+1)=next;
        plan.margin=min(plan.margin,margin);
        state=next;
        if norm(state(4:6))<=settings.stopSpeed
            plan.certified=true; return;
        end
    end
    safe=false;
end

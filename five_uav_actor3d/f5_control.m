function [base,info]=f5_control(state,refs,vrefs,waypoint,obstacles,p,hold)
% Original control3d APF/formation/pair equations, with all old ADP code absent.
    n=p.nUAV;base=zeros(3,n);
    info.tangentDegenerate=false(1,n);info.risk=zeros(1,n);
    for i=1:n
        pos=state(1:3,i);vel=state(4:6,i);
        if i==1
            if hold
                attraction=p.kHold*(p.target-pos)-p.kHoldD*vel;goal=p.target;
            else
                delta=waypoint-pos;distance=norm(delta);
                desiredVelocity=min(p.vmax,sqrt(2*0.6*p.amax*distance))*delta/max(distance,eps);
                attraction=p.navAccel*(desiredVelocity-vel);goal=waypoint;
            end
        else
            kp=p.kForm*(1+p.formGainBoost*hold);kd=p.kFormD*(1+p.formDampBoost*hold);
            attraction=kp*(refs(:,i)-pos)+kd*(vrefs(:,i)-vel);goal=refs(:,i);
        end
        [rep,risk,degenerate]=apf_repulsion3d(pos,goal,obstacles,p);
        pair=zeros(3,1);
        for j=1:n
            if j==i,continue;end
            diff=pos-state(1:3,j);d=norm(diff);
            if d<1e-9,direction=[cos(2*pi*i/n);sin(2*pi*i/n);0];else,direction=diff/d;end
            if d<p.uavAvoidSafe,pair=pair+p.kUAV*(p.uavAvoidSafe-d)/p.uavAvoidSafe*direction;end
            closing=dot(vel-state(4:6,j),direction);
            if closing<0 && d<p.uavAvoidSafe+2,pair=pair-p.kUAVVel*closing*direction;end
        end
        wObs=p.wObsMax*sqrt(risk);base(:,i)=(1-wObs)*attraction+wObs*rep+pair;
        if degenerate,base(:,i)=base(:,i)-0.5*vel;end
        info.tangentDegenerate(i)=degenerate;info.risk(i)=risk;
    end
end

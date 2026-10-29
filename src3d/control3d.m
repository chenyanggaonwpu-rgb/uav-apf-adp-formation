function [base,adpCorrection,info] = control3d(state,refs,vrefs,waypoint,obstacles,p,hold)
% CONTROL3D APF and formation act in parallel; shared ADP adds a correction.
    n=p.nUAV;
    base=zeros(3,n);adpCorrection=zeros(3,n);
    info.risk=zeros(1,n);info.tangentDegenerate=false(1,n);
    info.critic=zeros(1,n);
    info.adpContext=struct('position',refs,'velocity',vrefs,'trackPosition',true(1,n));
    info.adpContext.obstacles=obstacles;
    info.adpContext.positionRate=vrefs;
    tracking=isfield(p.adp,'mode') && ismember(p.adp.mode,{'tracking_value_v1','tracking_value_v2','tracking_value_v3','tracking_value_v4'});
    for i=1:n
        pos=state(1:3,i);vel=state(4:6,i);
        if i==1
            if hold
                attraction=p.kHold*(p.target-pos)-p.kHoldD*vel;
                goal=p.target;
                info.adpContext.position(:,i)=p.target;
                info.adpContext.velocity(:,i)=zeros(3,1);
                info.adpContext.positionRate(:,i)=zeros(3,1);
            else
                delta=waypoint-pos;
                distance=norm(delta);
                desiredVelocity=min(p.vmax,sqrt(2*0.6*p.amax*distance))* ...
                    delta/max(distance,eps);
                attraction=p.navAccel*(desiredVelocity-vel);
                goal=waypoint;
                info.adpContext.position(:,i)=pos;
                info.adpContext.velocity(:,i)=desiredVelocity;
                info.adpContext.trackPosition(i)=false;
                info.adpContext.positionRate(:,i)=zeros(3,1);
            end
        else
            kp=p.kForm*(1+p.formGainBoost*hold);
            kd=p.kFormD*(1+p.formDampBoost*hold);
            attraction=kp*(refs(:,i)-pos)+kd*(vrefs(:,i)-vel);
            goal=refs(:,i);
        end
        [rep,risk,degenerate]=apf_repulsion3d(pos,goal,obstacles,p);
        pair=zeros(3,1);
        for j=1:n
            if j==i, continue; end
            diff=pos-state(1:3,j);d=norm(diff);
            if d<1e-9
                direction=[cos(2*pi*i/n);sin(2*pi*i/n);0];
            else
                direction=diff/d;
            end
            if d<p.uavAvoidSafe
                pair=pair+p.kUAV*(p.uavAvoidSafe-d)/p.uavAvoidSafe*direction;
            end
            closing=dot(vel-state(4:6,j),direction);
            if closing<0 && d<p.uavAvoidSafe+2
                pair=pair-p.kUAVVel*closing*direction;
            end
        end
        wObs=p.wObsMax*sqrt(risk);
        base(:,i)=(1-wObs)*attraction+wObs*rep+pair;
        if degenerate
            base(:,i)=base(:,i)-0.5*vel; % reduce speed until replanning
        end
        if tracking
            [criticPhi,phi]=adp_tracking_features3d(state,i,info.adpContext,p);
        else
            phi=adp_features3d(state,i,refs,obstacles,p);criticPhi=phi;
        end
        info.critic(i)=p.adp.wC'*criticPhi;
        agentEnabled=~tracking || ~isfield(p.adp,'controlLeader') || p.adp.controlLeader || i~=p.leaderIdx;
        if p.adp.enable && agentEnabled
            adpCorrection(:,i)=p.adp.gain*limit_vector3d(p.adp.wA*phi,p.adp.uMax);
            if tracking
                adpCorrection(:,i)=adp_residual_limit3d(adpCorrection(:,i),base(:,i),p);
            end
        end
        info.risk(i)=risk;
        info.tangentDegenerate(i)=degenerate;
    end
end

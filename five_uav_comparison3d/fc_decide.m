function [waypoint,controller,diagnostic] = fc_decide(controller,history,time,obstacles,p,cfg,s,actorNet,method)
% history is executed 6-by-N state history, spaced by the original physical dt.
% Uses no case name, wall axis, witness path, geometric trap root, or future state.
    state=history(:,end); distance=norm(p.target-state(1:3));
    diagnostic=struct('stalled',false,'spread',NaN,'progress',NaN, ...
        'maximumSpeed',NaN,'exitCorridorSafe',false,'forwardSpeed',NaN, ...
        'action',zeros(3,1));
    waypoint=p.target;
    if controller.mode==5, return; end

    if controller.mode==3 && time-controller.modeSince>=s.cooldownSeconds-1e-9
        controller=transition(controller,1,'cooldown_complete');
        % Retain the already executed cooldown history for the next stall check.
    end
    if controller.mode~=2
        n=round(s.historySeconds/p.dt)+1;
        enough=size(history,2)>=n && time-controller.historySince>=s.historySeconds-1e-9;
        if enough && controller.mode~=3
            recent=history(:,end-n+1:end);
            diagnostic.spread=max(vecnorm(recent(1:3,:)-recent(1:3,1),2,1));
            distances=vecnorm(recent(1:3,:)-p.target,2,1);
            diagnostic.progress=distances(1)-distances(end);
            diagnostic.maximumSpeed=max(vecnorm(recent(4:6,:),2,1));
            diagnostic.stalled=diagnostic.spread<=s.stallSpreadMeters ...
                && abs(diagnostic.progress)<=s.stallProgressMeters ...
                && diagnostic.maximumSpeed<=s.stallMaxSpeed ...
                && ~(distance<=p.goalTol && norm(state(4:6))<=p.vTol);
        end
        if diagnostic.stalled
            if controller.activations>=s.maximumActivations
                controller.failureReason='activation_limit';
                controller=transition(controller,5,'activation_limit');
                return;
            end
            controller.activations=controller.activations+1;
            controller.ruleDirection=zeros(3,1);
            controller.entryDistance=distance; controller.clearSince=NaN;
            controller=transition(controller,2,'sustained_stall');
        elseif controller.mode==1 && distance<=s.terminalRadiusMeters && terminalClear()
            controller=transition(controller,4,'clear_terminal_approach');
        end
    end

    if controller.mode==2
        activeSeconds=time-controller.modeSince;
        if activeSeconds>=s.minimumActiveSeconds-1e-9
            if distance<=s.terminalRadiusMeters && terminalClear()
                controller.historySince=time;
                controller=transition(controller,4,'terminal_handoff');
                return;
            end
            direction=(p.target-state(1:3))/max(distance,eps);
            endpoint=state(1:3)+min(distance,s.exitCorridorMeters)*direction;
            strictP=p; strictP.obsSafe=p.obsSafe+s.exitExtraClearance;
            diagnostic.exitCorridorSafe=adp3d_certify_segment( ...
                state(1:3),endpoint,obstacles,strictP,cfg.safety);
            command=adp3d_leader_command(state,p.target,obstacles,p);
            predicted=motion_step3d(state,command,zeros(3,1),p);
            diagnostic.forwardSpeed=dot(predicted(4:6),direction);
            good=diagnostic.exitCorridorSafe ...
                && diagnostic.forwardSpeed>=s.exitMinimumForwardSpeed ...
                && controller.entryDistance-distance>=s.exitMinimumProgress;
            if good
                if isnan(controller.clearSince), controller.clearSince=time; end
                if time-controller.clearSince>=s.exitClearSeconds-1e-9
                    controller.historySince=time;
                    controller=transition(controller,3,'local_apf_corridor_clear');
                    return;
                end
            else
                controller.clearSince=NaN;
            end
        end
        if activeSeconds>=s.maximumActiveSeconds-1e-9
            controller.failureReason='escape_timeout';
            controller=transition(controller,5,'escape_timeout');
            return;
        end
        observationClock=tic;
        observation=adp3d_observation(state,obstacles,p,cfg.scanRange);
        controller.observationSeconds=controller.observationSeconds+toc(observationClock);
        proposalClock=tic;
        if strcmp(method,'actor')
            action=adp3d_actor_action(actorNet,observation);
            controller.actorCalls=controller.actorCalls+1;
        else
            assert(strcmp(method,'rule'));
            [action,controller.ruleDirection]=fc_rule_action(observation,cfg,p.obsSafe,controller.ruleDirection);
            controller.ruleCalls=controller.ruleCalls+1;
        end
        controller.proposalSeconds=controller.proposalSeconds+toc(proposalClock);
        [waypoint,~]=adp3d_decode_action(state,action,cfg.maxOffset);
        diagnostic.action=action;
        controller.proposalCalls=controller.proposalCalls+1;
    end

    function yes=terminalClear()
        yes=adp3d_certify_segment(state(1:3),p.target,obstacles,p,cfg.safety);
    end
    function updated=transition(updated,newMode,reason)
        event=struct('time',time,'from',updated.mode,'to',newMode,'reason',reason, ...
            'goalDistance',distance,'speed',norm(state(4:6)),'position',state(1:3));
        updated.events(end+1)=event;
        updated.mode=newMode; updated.modeSince=time;
    end
end

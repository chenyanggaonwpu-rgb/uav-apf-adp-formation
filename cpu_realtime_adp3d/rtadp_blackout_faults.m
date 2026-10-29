function rows=rtadp_blackout_faults(cfg,output)
% Explicit discrete plant experiment: late low-level control holds the previous
% acceleration. The UAV continues to move; an invalid braking certificate is recorded.
    variants={'no_blackout','main_blackout_100ms','main_blackout_300ms','actor_blackout_APF_continues'};
    p=adp3d_setup(cfg); scene=scenario3d('S11_blocked',0,p); collected={};
    for variantIndex=1:numel(variants)
        variant=variants{variantIndex}; state=[33.8;40;10;3;0;0];
        [backup,ok]=adp3d_braking_plan(state,scene.obstacles,p,cfg.safety); assert(ok);
        states=state; commands=[]; interventions=0; collision=false; minimum=inf;
        viabilityLost=false; heldSteps=0; previousCommand=zeros(3,1); unsafeSegmentCount=0;
        for k=1:100
            skip=strcmp(variant,'main_blackout_100ms') && ismember(k,2:3) ...
                || strcmp(variant,'main_blackout_300ms') && ismember(k,2:7);
            if skip
                command=previousCommand;
                next=motion_step3d(state,command,zeros(3,1),p); heldSteps=heldSteps+1;
                [safe,~]=adp3d_certify_segment(state(1:3),next(1:3),scene.obstacles,p,cfg.safety);
                unsafeSegmentCount=unsafeSegmentCount+~safe;
                backup=[];
            else
                if isempty(backup) || isempty(backup.commands) || norm(backup.states(:,1)-state)>1e-8
                    [backup,ok]=adp3d_braking_plan(state,scene.obstacles,p,cfg.safety);
                    if ~ok, viabilityLost=true; end
                end
                if viabilityLost
                    command=-state(4:6)/p.dt;
                    next=motion_step3d(state,command,zeros(3,1),p);
                    backup=[];
                else
                    [next,backup,info]=adp3d_safe_step(state,[3;0;0],backup,scene.obstacles,p,cfg.safety);
                    command=info.executedCommand; interventions=interventions+info.intervention;
                end
            end
            [hit,distance]=check_swept3d(state,next,scene.obstacles,p);
            collision=collision || hit; minimum=min(minimum,distance);
            states(:,end+1)=next; commands(:,end+1)=command; %#ok<AGROW>
            state=next; previousCommand=command;
            % Continue physical emergency braking to zero, rather than truncate before impact.
            if viabilityLost && norm(state(4:6))<=cfg.safety.stopSpeed, break; end
        end
        summary=struct('variant',variant,'heldControlSteps',heldSteps, ...
            'collision',collision,'lostBrakingViability',viabilityLost, ...
            'minimumObstacleDistance',minimum,'hardMargin',minimum-p.obsSafe, ...
            'finalSpeed',norm(state(4:6)),'interventions',interventions, ...
            'uncertifiedHeldSegments',unsafeSegmentCount,'physicalSteps',size(commands,2), ...
            'expectedNegativeControl',ismember(variantIndex,[2 3]));
        assert(norm(states(1:3,2)-states(1:3,1))>0,'RTADP:PlantFrozen');
        if ismember(variantIndex,[1 4]), assert(~collision && ~viabilityLost && interventions>0); end
        if ismember(variantIndex,[2 3]), assert(heldSteps>0 && viabilityLost && collision); end
        save(fullfile(output,['blackout_' variant '.mat']),'states','commands','summary','p','scene','-v7');
        collected{end+1}=summary; %#ok<AGROW>
        fprintf('FAULT %s held=%d hit=%d viabilityLost=%d hardMargin=%.4f\n', ...
            variant,heldSteps,collision,viabilityLost,summary.hardMargin);
    end
    rows=struct2table([collected{:}]); writetable(rows,fullfile(output,'blackout_faults.csv'));
end

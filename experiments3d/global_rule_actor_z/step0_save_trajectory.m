function step0_save_trajectory(result,caseId,arm,outDir)
% Save the Step-0 trajectory format used by the plotting tools.
    p=result.params;
    if isfield(result.history,'state') && ~isempty(result.history.state)
        lead=reshape(result.history.state(1:3,1,:),3,[]);
        t=result.history.t;
        minObs=min(result.history.obstacleClearance);
        minPair=min(result.history.pairDistance);
    else
        lead=result.initialFleetState(1:3,1);
        t=0;
        minObs=NaN;
        minPair=NaN;
    end
    boxes=zeros(0,4);
    for obstacle=result.scene.obstacles
        if strcmp(obstacle.type,'box')
            center=obstacle.center(1:2);
            halfSize=obstacle.halfSize(1:2);
            lower=center(:)-halfSize(:);
            upper=center(:)+halfSize(:);
            boxes(end+1,:)=[lower(1),upper(1),lower(2),upper(2)]; %#ok<AGROW>
        end
    end
    start=p.start;goal=p.target;ws=p.workspace;
    success=result.success;reason=result.reason;
    raw=fullfile(outDir,'raw');
    if ~isfolder(raw),mkdir(raw);end
    save(fullfile(raw,[caseId '.mat']),'lead','t','boxes','start','goal', ...
        'success','reason','arm','ws','minObs','minPair');
    fprintf('SAVE %s %s %s success=%d path=%.1f\n',arm,caseId,reason,double(success), ...
        sum(vecnorm(diff(lead,1,2))));
end


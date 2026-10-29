function [action,direction,detail]=fc_rule_action(observation,cfg,bodyMargin,previousDirection)
% Deterministic local free-ray waypoint heuristic. No map/case metadata input.
% The same 32-vector as the Actor: goal direction, velocity, 26 ideal ranges.
% Constants are declared before pilot and formal flights; no outcome tuning.
    persistent directions
    if isempty(directions)
        [x,y,z]=ndgrid(-1:1,-1:1,-1:1);grid=[x(:),y(:),z(:)]';
        axesDirections=[1,-1,0,0,0,0;0,0,1,-1,0,0;0,0,0,0,1,-1];
        directions=[axesDirections,grid(:,sum(grid~=0,1)>1)];
        directions=directions./vecnorm(directions);
    end
    assert(isequal(size(observation),[32,1]) && all(isfinite(observation)));
    goal=double(observation(1:3));goal=goal/max(norm(goal),eps);
    ranges=cfg.scanRange*double(observation(7:32))';
    usable=max(0,min(cfg.maxOffset,.85*(ranges-bodyMargin-.25)));
    score=usable/cfg.maxOffset+.65*(goal'*directions)+.25*(previousDirection'*directions);
    score(usable<.75)=-inf;
    if all(isinf(score)),[~,beam]=max(usable);else,[~,beam]=max(score);end
    direction=directions(:,beam);
    action=direction*(usable(beam)/cfg.maxOffset);
    assert(norm(action)<=1+1e-12 && all(abs(action)<=1));
    detail=struct('selectedBeam',beam,'rangeMeters',ranges(beam),'offsetMeters',usable(beam));
end

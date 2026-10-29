function cases=fc_cases(p,cfg,phase)
% Seeds and geometry families fixed before any formal controller is evaluated.
    cases=cell(0,1);
    if strcmp(phase,'pilot')
        normal=77001;structured=77011;windows=77021;repeated=77031;concave=77041;negative=77051;
    else
        normal=71001:71048;structured=72001:72024;windows=73001:73032;
        repeated=74001:74024;concave=75001:75008;negative=76001:76008;
    end
    for seed=normal
        q=p;q.seed=seed;scene=normal_scene3d(seed,q);
        add(sprintf('normal_%d',seed),'normal',q,scene);
        fprintf('FC_MAP normal=%d kind=%s\n',seed,scene.kind);
    end
    for seed=structured
        q=p;q.seed=seed;scene=scenario3d('random_structured',seed,q);
        add(sprintf('structured_%d',seed),'structured',q,scene);
    end
    for seed=windows
        stress=adp3d_make_case(seed,cfg,p);q=stress.parameters;q.seed=seed;
        scene=makeScene(sprintf('window_%d',seed),'single_window',seed,stress.obstacles);
        add(scene.id,'single_window',q,scene);
        cases{end}.geometryEvidence=rmfield(stress,{'parameters','initialBackup','obstacles'});
    end
    for seed=repeated
        stream=RandStream('mt19937ar','Seed',seed);axisIndex=randi(stream,2);direction=2*randi(stream,2)-3;
        firstLo=8+2*rand(stream);firstHi=18+2*rand(stream);
        if rand(stream)<.5,z=firstLo;else,z=firstHi;end
        if rand(stream)<.5,secondLo=2+3*rand(stream);secondHi=11+2*rand(stream);
        else,secondLo=14+2*rand(stream);secondHi=25+2*rand(stream);end
        firstX=23+3*rand(stream);secondX=49+3*rand(stream);
        obs=[wall(firstX,firstX+5+rand(stream),firstLo,firstHi), ...
            wall(secondX,secondX+5+rand(stream),secondLo,secondHi)];
        q=p;q.seed=seed;side=30+20*rand(stream);
        q.start=[8;side;z];q.target=[73;side;z];
        [q,obs]=orient(q,obs,axisIndex,direction);
        scene=makeScene(sprintf('double_%d',seed),'double_window',seed,obs);
        add(scene.id,'double_window',q,scene);
    end
    for seed=concave
        stream=RandStream('mt19937ar','Seed',seed);axisIndex=randi(stream,2);direction=2*randi(stream,2)-3;
        side=35+10*rand(stream);half=12+4*rand(stream);height=20+4*rand(stream);
        front=29+3*rand(stream);back=49+3*rand(stream);thick=2+rand(stream);
        obs=[obstacle3d('box',[front;side-half;0],[back;side-half+thick;height]), ...
            obstacle3d('box',[front;side+half-thick;0],[back;side+half;height]), ...
            obstacle3d('box',[back-thick;side-half;0],[back;side+half;height])];
        q=p;q.seed=seed;q.start=[12;side;10];q.target=[70;side;10];
        [q,obs]=orient(q,obs,axisIndex,direction);
        scene=makeScene(sprintf('concave_%d',seed),'concave',seed,obs);add(scene.id,'concave',q,scene);
    end
    for seed=negative
        stress=adp3d_make_case(seed,cfg,p);q=stress.parameters;q.seed=seed;
        lo=[0;0;0];hi=[80;80;30];lo(stress.axisIndex)=stress.wallLo;hi(stress.axisIndex)=stress.wallHi;
        scene=makeScene(sprintf('blocked_%d',seed),'blocked',seed,obstacle3d('box',lo,hi));
        add(scene.id,'blocked',q,scene);
    end
    function add(id,family,q,scene)
        cases{end+1,1}=struct('id',id,'family',family,'seed',q.seed,'parameters',q,'scene',scene);
    end
end
function scene=makeScene(id,family,seed,obstacles)
    scene=struct('id',id,'family',family,'seed',seed,'kind','comparison_predeclared','obstacles',obstacles);
end
function obs=wall(lo,hi,gapLo,gapHi)
    obs=[obstacle3d('box',[lo;0;0],[hi;80;gapLo]),obstacle3d('box',[lo;0;gapHi],[hi;80;30])];
end
function [q,obs]=orient(q,obs,axisIndex,direction)
    if direction<0
        q.start(1)=80-q.start(1);q.target(1)=80-q.target(1);
        for k=1:numel(obs),obs(k).center(1)=80-obs(k).center(1);end
    end
    if axisIndex==2
        q.start([1 2])=q.start([2 1]);q.target([1 2])=q.target([2 1]);
        for k=1:numel(obs),obs(k).center([1 2])=obs(k).center([2 1]);obs(k).halfSize([1 2])=obs(k).halfSize([2 1]);end
    end
end

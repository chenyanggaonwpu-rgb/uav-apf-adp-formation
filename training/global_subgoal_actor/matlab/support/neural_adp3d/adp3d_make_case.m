function item = adp3d_make_case(seed,cfg,baseParameters)
% 明确构造三维窗口墙 APF 平衡点；不是从随机地图里筛出成功样本。
% seed=0 为原 S8_window 的 y 对齐单机例；其他 seed 随机墙向、厚度、窗口和进近侧。
    p=baseParameters;
    if seed==0
        axisIndex=1; direction=1; wallLo=36; wallHi=44;
        gapLo=10; gapHi=20; lateral=62; startZ=10;
    else
        stream=RandStream('mt19937ar','Seed',seed);
        axisIndex=randi(stream,2);
        direction=2*randi(stream,2)-3;
        wallLo=32+8*rand(stream);
        wallHi=wallLo+6+4*rand(stream);
        gapLo=8+4*rand(stream);
        gapHi=gapLo+8+4*rand(stream);
        lateral=20+40*rand(stream);
        if rand(stream)<0.5, startZ=gapLo; else, startZ=gapHi; end
    end
    otherAxis=3-axisIndex;
    a=[0;0;0]; b=[80;80;gapLo];
    a(axisIndex)=wallLo; b(axisIndex)=wallHi;
    lower=obstacle3d('box',a,b);
    a=[0;0;gapHi]; b=[80;80;30];
    a(axisIndex)=wallLo; b(axisIndex)=wallHi;
    upper=obstacle3d('box',a,b);
    obstacles=[lower upper];
    p.target=zeros(3,1);
    p.target(otherAxis)=lateral;
    p.target(3)=startZ;
    if direction>0, face=wallLo; p.target(axisIndex)=70;
    else, face=wallHi; p.target(axisIndex)=10; end
    state=zeros(6,1); state(otherAxis)=lateral; state(3)=startZ;
    left=p.obsSafe+0.05; right=p.rho0-0.05;
    fl=normalCommand(left); fr=normalCommand(right);
    assert(fl<0 && fr>0,'ADP3D:NoEquilibriumBracket');
    for k=1:60
        middle=(left+right)/2;
        if normalCommand(middle)>0, right=middle; else, left=middle; end
    end
    state(axisIndex)=face-direction*(left+right)/2;
    residual=adp3d_leader_command(state,p.target,obstacles,p);
    assert(norm(residual)<1e-8,'ADP3D:NotAnAPFEquilibrium');
    [backup,canStop]=adp3d_braking_plan(state,obstacles,p,cfg.safety);
    assert(canStop,'ADP3D:UnsafeInitialCase');
    mid=state(1:3); mid(3)=(gapLo+gapHi)/2;
    far=mid;
    if direction>0, far(axisIndex)=wallHi+7;
    else, far(axisIndex)=wallLo-7; end
    witness=[state(1:3) mid far p.target];
    witnessMargin=inf;
    for k=1:size(witness,2)-1
        [ok,margin]=adp3d_certify_segment(witness(:,k),witness(:,k+1),obstacles,p,cfg.safety);
        assert(ok,'ADP3D:NoGeometricWitness');
        witnessMargin=min(witnessMargin,margin);
    end
    p.start=state(1:3);
    item=struct('seed',seed,'family','window-wall','parameters',p, ...
        'obstacles',obstacles,'initialState',state,'initialBackup',backup, ...
        'apfResidualNorm',norm(residual),'witness',witness, ...
        'witnessMarginLowerBound',witnessMargin,'axisIndex',axisIndex, ...
        'direction',direction,'wallLo',wallLo,'wallHi',wallHi, ...
        'gapLo',gapLo,'gapHi',gapHi);

    function value=normalCommand(distance)
        candidate=state;
        candidate(axisIndex)=face-direction*distance;
        u=adp3d_leader_command(candidate,p.target,obstacles,p);
        value=direction*u(axisIndex);
    end
end

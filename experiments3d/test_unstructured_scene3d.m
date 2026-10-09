function test_unstructured_scene3d()
% TEST_UNSTRUCTURED_SCENE3D Reproducibility, independence and unscreened starts.
    root=fileparts(fileparts(mfilename('fullpath'))); addpath(fullfile(root,'config'),fullfile(root,'src3d'));
    p=default_params3d(); counts=[]; kinds=strings(0,1); occupied=0; nominal=0;
    options=struct('mode','unconditioned'); % The historical unscreened control is explicit.
    for seed=2001:2200
        [a,m]=unstructured_scene3d(seed,p,options); [b,n]=unstructured_scene3d(seed,p,options);
        assert(isequaln(a,b) && isequaln(m,n));
        moved=p; moved.start=[60;10;23]; moved.target=[20;60;7];
        c=unstructured_scene3d(seed,moved,options); assert(isequaln(a,c),'Geometry must not depend on mission endpoints.');
        assert(m.placementAttempts==m.obstacleCount && m.rejectedObstacles==0);
        assert(numel(a.obstacles)==m.obstacleCount && m.obstacleCount>=1 && m.obstacleCount<=40);
        counts(end+1)=m.obstacleCount; %#ok<AGROW>
        state=init_formation3d(p); initialClear=inf;
        for o=a.obstacles
            kinds(end+1)=string(o.type); %#ok<AGROW>
            if strcmp(o.type,'sphere')
                c=o.center; half=o.r*ones(3,1); assert(o.r>=1 && o.r<=6);
            elseif strcmp(o.type,'cylinder')
                c=[o.center;(o.zmin+o.zmax)/2];half=[o.r;o.r;(o.zmax-o.zmin)/2];
                assert(o.r>=1 && o.r<=6 && 2*half(3)>=2 && 2*half(3)<=24);
            else
                c=o.center;half=o.halfSize; assert(all(2*half>=[2;2;2]) && all(2*half<=[20;20;24]));
            end
            assert(all(c-half>=p.workspace(1,:)'-1e-12) && all(c+half<=p.workspace(2,:)'+1e-12));
            for j=1:p.nUAV, initialClear=min(initialClear,obstacle_distance3d(state(1:3,j),o));end
        end
        occupied=occupied+(initialClear<=p.obsSafe); nominal=nominal+m.nominalVolume;
    end
    assert(numel(unique(counts))>20 && numel(unique(kinds))==3);
    assert(occupied>0,'The declared unconditioned generator must retain occupied starts.');
    % Runner must accept legitimate early exits without mislabelling execution errors.
    p.seed=999999; blocked=struct('id','occupied','kind','random','seed',p.seed, ...
        'obstacles',obstacle3d('sphere',p.start,2),'family','unstructured','description','Early-exit fixture.');
    r=run_simulation3d(p,blocked);assert(~r.success && strcmp(r.reason,'invalid_scene'));
    fprintf('UNSTRUCTURED_TEST_PASSED maps=200 distinct_counts=%d occupied_initial=%d nominal_sum=%.6g\n',numel(unique(counts)),occupied,nominal);
end

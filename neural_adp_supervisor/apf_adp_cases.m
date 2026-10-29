function cases = apf_adp_cases(split,cfg,s)
% Predeclared construction only. No rollout, rejection by outcome, or label in control.
    base=adp3d_setup(cfg); cases={};
    switch split
        case 'pilot'
            trapSeeds=10001:10004; normalSeeds=40001:40008;
            legacy={'S1_sphere','S2_cylinder','S6_terminal','S7_underpass'};
        case 'development'
            trapSeeds=10001:10016; normalSeeds=40001:40024;
            legacy={'S1_sphere','S2_cylinder','S3_overpass','S4_space_s', ...
                'S5_choice','S6_terminal','S7_underpass','S9_altitude_choice','S10_layer_gap'};
        case 'heldout'
            trapSeeds=20001:20016; normalSeeds=50001:50024;
            legacy={};
        otherwise, error('APFADP:Split','Unknown split');
    end
    for seed=trapSeeds
        trap=adp3d_make_case(seed,cfg,base); p=trap.parameters;
        initial=trap.initialState;
        if trap.direction>0, initial(trap.axisIndex)=10; else, initial(trap.axisIndex)=70; end
        p.start=initial(1:3);
        add(sprintf('approach_window_%d',seed),'approach_window',seed,p,trap.obstacles,initial);
        % Offline plotting metadata. Never passed to the controller.
        cases{end}.wallMetadata=rmfield(trap,{'parameters','obstacles','initialBackup'});
    end
    for seed=normalSeeds
        stream=RandStream('mt19937ar','Seed',seed);
        familyIndex=mod(seed-1,8)+1; p=base; initial=zeros(6,1);
        empty=scenario3d('S0_empty',0,p); obstacles=empty.obstacles;
        lateral=25+25*rand(stream); z=10+8*rand(stream);
        initial(1:3)=[10;lateral;z]; p.target=[70;lateral;z];
        switch familyIndex
            case 1
                family='empty_diagonal'; p.target=[68;65;22];
            case 2
                family='empty_vertical'; initial(1:3)=[25;lateral;4]; p.target=[25;lateral;26];
            case 3
                family='short_terminal'; p.target=initial(1:3)+[2+rand(stream);1;0.5];
            case 4
                family='braking_terminal'; initial(4:6)=[2;0;0]; p.target=initial(1:3)+[4;0;0];
            case 5
                family='window_center'; initial(3)=15; p.target(3)=15;
                obstacles=[obstacle3d('box',[34;0;0],[43;80;9]), ...
                    obstacle3d('box',[34;0;21],[43;80;30])];
            case 6
                family='sphere_detour';
                obstacles=obstacle3d('sphere',[39;lateral+2.2+rand(stream);z],3.5);
            case 7
                family='cylinder_detour';
                obstacles=obstacle3d('cylinder',[40;lateral+2.5+rand(stream)],3.5,0,24);
            case 8
                family='box_detour';
                obstacles=obstacle3d('box',[36;lateral+1.5;3],[43;lateral+8;24]);
        end
        p.start=initial(1:3);
        add(sprintf('%s_%d',family,seed),family,seed,p,obstacles,initial);
    end
    for k=1:numel(legacy)
        p=base; scene=scenario3d(legacy{k},0,p);
        add(['legacy_' legacy{k}],'legacy',0,p,scene.obstacles,[p.start;zeros(3,1)]);
    end
    if ~strcmp(split,'heldout')
        p=base; p.start=[10;40;10]; p.target=[70;40;10];
        scene=scenario3d('S11_blocked',0,p);
        add('blocked_negative_control','infeasible',0,p,scene.obstacles,[p.start;zeros(3,1)]);
        % Two separated traps exercise real exit and later reactivation, not a timer schedule.
        for k=1:2
            p=base; p.start=[8;40;10]; p.target=[73;40;10];
            if k==1, secondGap=[10 20]; else, secondGap=[2 10]; end
            obstacles=[obstacle3d('box',[25;0;0],[31;80;10]), ...
                obstacle3d('box',[25;0;20],[31;80;30]), ...
                obstacle3d('box',[52;0;0],[58;80;secondGap(1)]), ...
                obstacle3d('box',[52;0;secondGap(2)],[58;80;30])];
            add(sprintf('sequential_windows_%d',k),'sequential_windows',k,p,obstacles,[p.start;zeros(3,1)]);
        end
    end
    for k=1:numel(cases)
        [~,ok]=adp3d_braking_plan(cases{k}.initialState,cases{k}.obstacles, ...
            cases{k}.parameters,cfg.safety);
        assert(ok,'APFADP:UnsafeCase','Unsafe initial case %s',cases{k}.id);
    end
    function add(id,family,seed,p,obstacles,initialState)
        p.Tmax=s.maximumFlightSeconds;
        cases{end+1}=struct('id',id,'family',family,'seed',seed,'split',split, ...
            'parameters',p,'obstacles',obstacles,'initialState',initialState);
    end
end

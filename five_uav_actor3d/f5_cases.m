function cases=f5_cases(phase,p,cfg)
% Catalogue fixed before flight; no controller-success filtering of maps.
    cases=cell(0,1);
    if strcmp(phase,'pilot')
        fixed={'S0_empty','S1_sphere','S8_window'};
        normal=60001:60002;structured=60011;windows=60021:60023;negative=[];
    else
        fixed={'S0_empty','S1_sphere','S2_cylinder','S3_overpass','S4_space_s', ...
            'S5_choice','S6_terminal','S7_underpass','S8_window','S9_altitude_choice', ...
            'S10_layer_gap','S11_blocked'};
        normal=61001:61048;structured=62001:62024;windows=63001:63040;negative=64001:64004;
    end
    for i=1:numel(fixed)
        q=p;q.seed=59000+i;scene=scenario3d(fixed{i},q.seed,q);
        cases{end+1,1}=item(fixed{i},'fixed_global',q,scene,'global');
    end
    for seed=normal
        q=p;q.seed=seed;
        [scene,~,~]=normal_scene3d(seed,q);
        cases{end+1,1}=item(sprintf('normal_%d',seed),'normal_global',q,scene,'global');
        fprintf('F5_MAP normal seed=%d kind=%s obstacles=%d\n',seed,scene.kind,numel(scene.obstacles));
    end
    for seed=structured
        q=p;q.seed=seed;scene=scenario3d('random_structured',seed,q);
        cases{end+1,1}=item(sprintf('structured_%d',seed),'structured_global',q,scene,'global');
    end
    for seed=windows
        stress=adp3d_make_case(seed,cfg,p);q=stress.parameters;q.seed=seed;
        scene=struct('id',sprintf('window_%d',seed),'kind','explicit_stress', ...
            'seed',seed,'family','window_local_apf','obstacles',stress.obstacles);
        cases{end+1,1}=item(scene.id,'window_local_apf',q,scene,'local_apf');
    end
    for seed=negative
        stress=adp3d_make_case(seed,cfg,p);q=stress.parameters;q.seed=seed;
        lo=[0;0;0];hi=[80;80;30];lo(stress.axisIndex)=stress.wallLo;hi(stress.axisIndex)=stress.wallHi;
        scene=struct('id',sprintf('blocked_%d',seed),'kind','negative_control', ...
            'seed',seed,'family','blocked_local_apf','obstacles',obstacle3d('box',lo,hi));
        cases{end+1,1}=item(scene.id,'blocked_local_apf',q,scene,'local_apf');
    end
end
function value=item(id,family,p,scene,policy)
    value=struct('id',id,'family',family,'seed',p.seed,'parameters',p,'scene',scene,'routePolicy',policy);
end

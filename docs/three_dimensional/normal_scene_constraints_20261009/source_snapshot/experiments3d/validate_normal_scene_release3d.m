function validate_normal_scene_release3d(outputDir)
% VALIDATE_NORMAL_SCENE_RELEASE3D Focused geometry tests, existing regressions and three fixed smoke flights.
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'), ...
        fullfile(root,'tests'),fullfile(root,'experiments3d'));
    if ~isfolder(outputDir), mkdir(outputDir); end
    fprintf('NORMAL_VALIDATION_BEGIN output=%s\n',outputDir);
    focused=test_normal_scene3d(fullfile(outputDir,'focused'));
    test_unstructured_scene3d();
    original=readtable(fullfile(root,'docs','three_dimensional','margin_grid49_20261008','map_manifest.csv'), ...
        'TextType','string','VariableNamingRule','preserve');
    baseParameters=default_params3d();
    for k=1:height(original)
        raw=random_solids3d(original.seed(k),baseParameters);
        bytes=unicode2native(jsonencode(struct('workspace',baseParameters.workspace, ...
            'obstacles',raw.obstacles)),'UTF-8');
        digest=java.security.MessageDigest.getInstance('SHA-256'); digest.update(bytes);
        hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
        assert(strcmp(hash,original.mapHash(k)),'Historical raw geometry changed.');
    end
    fprintf('HISTORICAL_MAP_HASHES_PASSED maps=%d\n',height(original));
    suites={'test_3d','test_environment3d','test_adaptive_height3d','test_directional_clearance3d'};
    for k=1:numel(suites)
        fprintf('REGRESSION_BEGIN %s\n',suites{k}); feval(suites{k});
        fprintf('REGRESSION_PASSED %s\n',suites{k});
    end
    rows=struct([]);
    for seed=3001:3003
        frozen=load(fullfile(outputDir,'focused','maps',sprintf('seed_%06d.mat',seed)));
        q=frozen.q; scene=frozen.generated; q.animate=false;
        trialDir=fullfile(outputDir,sprintf('flight_%06d',seed)); mkdir(trialDir);
        save_scene_generation3d(scene,trialDir);
        fprintf('NORMAL_FLIGHT_BEGIN seed=%d\n',seed);
        if seed==3001
            q.sceneId='random_normal'; q.resultsRoot=trialDir;
            result=main3d(q); % Exercise the public entry and pre-flight persistence.
            assert(isequaln(scene,result.scene),'The public entry must reuse the deterministic map.');
        else
            result=run_simulation3d(q,scene);
        end
        save(fullfile(trialDir,'simulation_result.mat'),'result','q','scene','-v7.3');
        assert(~ismember(result.reason,{'invalid_scene','generation_failed'}), ...
            'Generated normal maps must be valid at simulation start.');
        rows=[rows;struct('seed',seed,'success',result.success,'reason',string(result.reason), ...
            'planStatus',string(result.plan.status),'time',result.time, ...
            'obstacleCount',numel(scene.obstacles))]; %#ok<AGROW>
        fprintf('NORMAL_FLIGHT_COMPLETE seed=%d reason=%s time=%.2f\n',seed,result.reason,result.time);
    end
    writetable(struct2table(rows),fullfile(outputDir,'smoke_flights.csv'));
    receipt=struct('passed',true,'version','2.3.3','focused',focused, ...
        'historicalRawGeneratorPassed',true,'historicalMapHashesMatched',height(original), ...
        'publicMainEntryPassed',true,'regressionSuites',{suites}, ...
        'smokeFlights',rows,'smokeFlightRule','Fixed fresh seeds 3001:3003; all failures retained; too small for a rate claim.', ...
        'generatedBeforeFlight',true,'outputsRoot',outputDir);
    file=fopen(fullfile(outputDir,'validation_receipt.json'),'w','n','UTF-8');
    fprintf(file,'%s',jsonencode(receipt,PrettyPrint=true)); fclose(file);
    fprintf('NORMAL_RELEASE_VALIDATION_PASSED\n');
end

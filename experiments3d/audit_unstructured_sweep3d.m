function audit_unstructured_sweep3d(studyDir)
% AUDIT_UNSTRUCTURED_SWEEP3D Check all maps, raw trials, gates and recorded timing.
    snap=fullfile(studyDir,'source_snapshot');addpath(fullfile(snap,'config'),fullfile(snap,'src3d'),fullfile(snap,'experiments3d'));
    stored=load(fullfile(studyDir,'protocol.mat'),'protocol');protocol=stored.protocol;
    manifest=readtable(fullfile(studyDir,'map_manifest.csv'),'Delimiter',',','TextType','string','VariableNamingRule','preserve');
    hashes=readtable(fullfile(studyDir,'source_sha256.csv'),'Delimiter',',','TextType','string','VariableNamingRule','preserve');
    for k=1:height(hashes),assert(strcmp(midpoint_sha256_file3d(fullfile(snap,char(hashes.file(k)))),hashes.sha256(k)));end
    records=struct([]);timings=struct([]);seen=[];clock=tic;
    for worker=1:protocol.nWorkers
        directory=fullfile(studyDir,'workers',sprintf('worker_%02d',worker));
        status=jsondecode(fileread(fullfile(directory,'status.json')));assert(strcmp(status.status,'complete'));
        opts=detectImportOptions(fullfile(directory,'trials.csv'),'Delimiter',',','VariableNamingRule','preserve');
        opts=setvartype(opts,{'mapHash','reason','planStatus','fixedPlanStatus','addedHeights'},'string');
        csv=readtable(fullfile(directory,'trials.csv'),opts);
        assert(height(csv)==protocol.ranges(worker,2)-protocol.ranges(worker,1)+1);
        for index=1:height(csv)
            seed=csv.seed(index);seen(end+1)=seed; %#ok<AGROW>
            rawFile=fullfile(directory,'trials',sprintf('random_%06d.mat',seed));d=load(rawFile);
            m=manifest(manifest.seed==seed,:);assert(height(m)==1);
            mapFile=fullfile(studyDir,char(m.mapFile));frozen=load(mapFile);
            assert(strcmp(midpoint_sha256_file3d(mapFile),m.mapMatSha256));
            assert(isequaln(d.q,frozen.q) && isequaln(d.scene,frozen.scene));
            expected=protocol.p;expected.seed=seed;expected.randomObstacleCount=m.obstacleCount;
            assert(isequaln(expected,d.q) && isequal(d.q.heightLayers,[5 10 15 20 25]));
            assert(d.q.pathClearance==4.2 && d.q.desiredClear==4 && ~d.q.directionalClearance.enable);
            assert(d.q.noiseStd==0 && d.q.disturbanceMax==0 && d.q.adp.enable && d.q.safety.enable);
            regenerated=unstructured_scene3d(seed,protocol.p,protocol.generation);assert(isequaln(regenerated,d.scene));
            bytes=unicode2native(jsonencode(struct('workspace',d.q.workspace,'obstacles',d.scene.obstacles)),'UTF-8');
            md=java.security.MessageDigest.getInstance('SHA-256');md.update(bytes);
            actualHash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
            assert(strcmp(actualHash,m.mapHash) && strcmp(actualHash,d.mapHash));
            assert(isempty(d.errorText) && ~isempty(d.r),'UAV3D:TrialError','Execution error in seed %d.',seed);
            names=csv.Properties.VariableNames;
            for k=1:numel(names)
                name=names{k};a=csv.(name)(index,:);b=d.row.(name);
                if ischar(b) || isstring(b)
                    assert((all(ismissing(a)) && strlength(string(b))==0) || strcmp(string(a),string(b)));
                else
                    assert((isnan(a)&&isnan(b)) || a==b || abs(a-b)<=2e-12*max(1,abs(b)));
                end
            end
            r=d.r;q=d.q;assert(r.success==d.row.success && strcmp(r.reason,d.row.reason));
            assert(r.success==strcmp(r.reason,'success'));
            initial=init_formation3d(q);initialMin=independent_min(initial(1:3,:),d.scene.obstacles);
            assert(abs(initialMin-m.initialMinObstacleClearance)<1e-10);
            assert((initialMin>q.obsSafe)==logical(m.initialHardSafe));
            assert(strcmp(r.reason,'invalid_scene')==~logical(m.initialHardSafe));
            assert(logical(d.row.initialHardSafe)==logical(m.initialHardSafe));
            assert(logical(d.row.leaderEndpointsPlanClear)==logical(m.leaderEndpointsPlanClear));
            routeClearance=nan;
            if isfield(r,'initialRoute') && ~isempty(r.initialRoute)
                points=[q.start,r.initialRoute];routeClearance=inf;
                for k=1:size(points,2)-1
                    routeClearance=min(routeClearance,segment_clearance3d(points(:,k),points(:,k+1),d.scene.obstacles));
                end
                assert(routeClearance>=q.pathClearance-1e-6);
            end
            if isfield(r.plan,'heightSelection')
                hs=r.plan.heightSelection;assert(numel(hs.addedHeights)<=q.adaptiveHeight.maxAddedLayers);
                for k=find(hs.decisions=="added_midpoint")'
                    centre=mean(hs.sections(k,3:4));assert(any(abs(hs.addedHeights-centre)<1e-8));
                    assert(~any(hs.baseLayers>=hs.sections(k,3)-1e-7 & hs.baseLayers<=hs.sections(k,4)+1e-7));
                end
            end
            vertexMin=nan;pairMin=nan;cycles=0;cycleMs=zeros(0,1);maxVelocity=nan;
            if isfield(r.history,'t') && ~isempty(r.history.t)
                h=r.history;cycles=numel(h.t);assert(all(h.noise==0,'all') && all(h.disturbance==0,'all'));
                vertexMin=independent_min(reshape(h.state(1:3,:,:),3,[]),d.scene.obstacles);pairMin=inf;
                for a=1:q.nUAV-1
                    for b=a+1:q.nUAV
                        delta=squeeze(h.state(1:3,a,:)-h.state(1:3,b,:));pairMin=min(pairMin,min(vecnorm(delta)));
                    end
                end
                assert(vertexMin>=min(h.obstacleClearance)-1e-7 && pairMin>=min(h.pairDistance)-1e-7);
                maxVelocity=max(sqrt(sum(h.state(4:6,:,:).^2,1)),[],'all');assert(maxVelocity<=q.vmax+1e-9);
                assert(r.time<=q.Tmax+1e-9 && abs(r.time-h.t(end))<1e-9);
                if r.success
                    count=ceil(q.holdDuration/q.dt-1e-10);final=h.state(:,:,end-count+1:end);
                    assert(all(vecnorm(squeeze(final(1:3,1,:))-q.target)<=q.goalTol+1e-9));
                    assert(all(sqrt(sum(final(4:6,:,:).^2,1))<=q.vTol+1e-9,'all'));
                    assert(all(h.formationError(end-count+1:end)<=q.formTol+1e-9));
                    assert(~any(h.referenceBlocked(end-count+1:end)));
                    assert(min(h.obstacleClearance)>q.obsSafe && min(h.pairDistance)>q.uavSafe);
                    assert(vertexMin>q.obsSafe && pairMin>q.uavSafe);
                end
                cycleMs=h.cycleMs(:);cycleMs=cycleMs(cycleMs>0);
            else
                assert(~r.success && ismember(string(r.reason),["invalid_scene","planning_failed"]));
            end
            assert(cycles==d.row.cycles);
            records=[records;struct('seed',seed,'success',r.success,'reason',string(r.reason), ...
                'mapHashPassed',true,'mapFileHashPassed',true,'regenerationPassed',true,'parametersPassed',true, ...
                'csvPassed',true,'initialHardSafe',logical(m.initialHardSafe),'initialGeometryPassed',true, ...
                'successHoldChecked',r.success,'successHoldPassed',r.success,'initialRouteClearance',routeClearance, ...
                'vertexMin',vertexMin,'pairVertexMin',pairMin,'maxVelocity',maxVelocity, ...
                'rawMatSha256',string(midpoint_sha256_file3d(rawFile)))]; %#ok<AGROW>
            timings=[timings;struct('seed',seed,'success',r.success,'initialPlanSeconds',r.timing.initialPlanSeconds, ...
                'measuredCycles',numel(cycleMs),'cyclesAboveDt',nnz(cycleMs>1000*q.dt), ...
                'medianCycleMs',median(cycleMs),'p95CycleMs',quantile_rank(cycleMs,.95), ...
                'p99CycleMs',quantile_rank(cycleMs,.99),'maxCycleMs',quantile_rank(cycleMs,1), ...
                'maxReplanSeconds',max([0;r.timing.replanSeconds(:)]), ...
                'maxTerminalPlanSeconds',max([0;r.timing.terminalPlanSeconds(:)]))]; %#ok<AGROW>
        end
        fprintf('UNSTRUCTURED_AUDIT worker=%d checked=%d\n',worker,numel(records));
    end
    assert(isequal(sort(seen),protocol.seeds));raw=sortrows(struct2table(records),'seed');timing=sortrows(struct2table(timings),'seed');
    writetable(raw,fullfile(studyDir,'raw_mat_audit.csv'));writetable(timing,fullfile(studyDir,'timing_by_seed.csv'));
    audit=struct('passed',true,'records',height(raw),'uniqueSeeds',numel(unique(raw.seed)), ...
        'success',nnz(raw.success),'failure',nnz(~raw.success),'sourceFiles',height(hashes), ...
        'allFrozenMapFilesUnchanged',true,'allMapsRegenerateExactly',true,'allParametersAndCsvPassed',true, ...
        'independentGeometryAndSuccessHoldPassed',true,'wallSeconds',toc(clock));
    write_json('raw_mat_audit.json',audit);
    summary=struct('records',height(timing),'planningCalls',nnz(isfinite(timing.initialPlanSeconds)), ...
        'measuredCycles',sum(timing.measuredCycles),'cyclesAboveDt',sum(timing.cyclesAboveDt), ...
        'cycleDeadlineMs',1000*protocol.p.dt,'initialPlanMedianSeconds',median(timing.initialPlanSeconds,'omitnan'), ...
        'initialPlanP95Seconds',quantile_rank(timing.initialPlanSeconds,.95), ...
        'initialPlanMaxSeconds',quantile_rank(timing.initialPlanSeconds,1), ...
        'perFlightP99MedianMs',median(timing.p99CycleMs,'omitnan'),'maxCycleMs',quantile_rank(timing.maxCycleMs,1), ...
        'maxReplanSeconds',max(timing.maxReplanSeconds),'maxTerminalPlanSeconds',max(timing.maxTerminalPlanSeconds), ...
        'quantileRule','Nearest rank within each run; median of per-flight P99 explicitly separate.', ...
        'timingScope',protocol.timingRule);
    write_json('timing_summary.json',summary);
    fprintf('UNSTRUCTURED_AUDIT_PASSED records=%d success=%d cycles=%d\n',height(raw),nnz(raw.success),sum(timing.measuredCycles));
    function write_json(name,value)
        fid=fopen(fullfile(studyDir,name),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value,PrettyPrint=true));fclose(fid);
    end
end
function value=quantile_rank(values,fraction)
    values=sort(values(isfinite(values)));value=nan;
    if ~isempty(values),value=values(max(1,ceil(fraction*numel(values))));end
end
function value=independent_min(points,obstacles)
    value=inf;
    for o=obstacles
        if strcmp(o.type,'sphere')
            ds=vecnorm(points-o.center)-o.r;
        elseif strcmp(o.type,'cylinder')
            radial=vecnorm(points(1:2,:)-o.center)-o.r;
            vertical=max(o.zmin-points(3,:),points(3,:)-o.zmax);
            ds=sqrt(max(radial,0).^2+max(vertical,0).^2)+min(max(radial,vertical),0);
        else
            offsets=abs(points-o.center)-o.halfSize;
            ds=sqrt(sum(max(offsets,0).^2,1))+min(max(offsets,[],1),0);
        end
        value=min(value,min(ds));
    end
end

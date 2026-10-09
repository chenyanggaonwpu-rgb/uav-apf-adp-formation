function audit=audit_new_map_seed_sweep3d(studyDir)
% AUDIT_NEW_MAP_SEED_SWEEP3D Read saved results; do not rerun any simulation.
% Cross-check every CSV record against its raw MAT and frozen protocol.
    saved=load(fullfile(studyDir,'protocol.mat'),'protocol');
    protocol=saved.protocol;
    assert(strcmp(protocol.p.version,'2.1.0'));
    assert(isequal(protocol.seeds,1:1000));
    assert(isequal(protocol.p.heightLayers,[5 10 15 20 25]));
    assert(protocol.p.noiseStd==0 && protocol.p.disturbanceMax==0);
    assert(protocol.p.adp.enable && protocol.p.safety.enable);
    families=["mixed","underpass","window","staggered_windows"];
    rawRows=struct([]); seen=[]; auditClock=tic;
    for worker=1:protocol.nWorkers
        workerDir=fullfile(studyDir,'workers',sprintf('worker_%02d',worker));
        status=jsondecode(fileread(fullfile(workerDir,'status.json')));
        assert(strcmp(status.status,'complete'));
        rows=readtable(fullfile(workerDir,'trials.csv'),'TextType','string');
        assert(height(rows)==diff(protocol.ranges(worker,:))+1);
        assert(isequal(rows.seed',protocol.ranges(worker,1):protocol.ranges(worker,2)));
        for index=1:height(rows)
            seed=rows.seed(index); trialId=sprintf('C_random_%06d',seed);
            trialFile=fullfile(workerDir,'trials',[trialId '.mat']);
            data=load(trialFile,'q','r','scene','row','errorText');
            expected=protocol.p; expected.seed=seed;
            assert(isequaln(data.q,expected),'UAV3D:AuditParameters','seed %d parameters changed.',seed);
            assert(data.scene.seed==seed && strcmp(data.scene.id,'random'));
            family=families(mod(seed-1,4)+1);
            assert(strcmp(data.scene.family,family));
            assert(strcmp(data.row.trialId,trialId));
            names=rows.Properties.VariableNames;
            for field=1:numel(names)
                name=names{field}; actual=rows.(name)(index,:); stored=data.row.(name);
                if ischar(stored) || isstring(stored)
                    assert(strcmp(string(actual),string(stored)), ...
                        'UAV3D:AuditCsv','seed %d field %s differs.',seed,name);
                else
                    actual=double(actual); stored=double(stored);
                    equal=(isnan(actual) && isnan(stored)) || actual==stored || ...
                        abs(actual-stored)<=2e-12*max(1,abs(stored));
                    assert(equal,'UAV3D:AuditCsv','seed %d field %s differs.',seed,name);
                end
            end
            if isempty(data.r)
                assert(strcmp(data.row.reason,'execution_error') && ~data.row.success);
                assert(~isempty(data.errorText));
                success=false; reason="execution_error"; cycles=0;
            else
                result=data.r;
                success=result.success; reason=string(result.reason);
                assert(success==data.row.success && reason==string(data.row.reason));
                assert(success==strcmp(reason,'success'));
                assert(isequal(result.params.heightLayers,[5 10 15 20 25]));
                assert(isequaln(result.scene,data.scene));
                cycles=0;
                if isfield(result.history,'t'), cycles=numel(result.history.t); end
                assert(cycles==data.row.cycles);
                assert(result.time==data.row.time);
                if cycles>0
                    assert(all(result.history.noise==0,'all'));
                    assert(all(result.history.disturbance==0,'all'));
                    assert(nnz(result.history.safetyTriggered)==data.row.safetyTriggers);
                end
                if success
                    assert(cycles>0 && isempty(data.errorText));
                    holdCount=ceil(data.q.holdDuration/data.q.dt-1e-10);
                    final=result.history.state(:,:,end-holdCount+1:end);
                    leaderError=vecnorm(squeeze(final(1:3,1,:))-data.q.target);
                    speed=sqrt(sum(final(4:6,:,:).^2,1));
                    assert(all(leaderError<=data.q.goalTol+1e-10));
                    assert(all(speed<=data.q.vTol+1e-10,'all'));
                    assert(all(result.history.formationError(end-holdCount+1:end)<=data.q.formTol+1e-10));
                    assert(~any(result.history.referenceBlocked(end-holdCount+1:end)));
                    assert(min(result.history.obstacleClearance)>data.q.obsSafe);
                    assert(min(result.history.pairDistance)>data.q.uavSafe);
                end
            end
            % A missed full-width gate proves a fixed-layer graph obstruction.
            % It does not prove that the complete continuous-space flight is feasible.
            gateCount=0; gatesWithoutLayer=0; gateCenters=nan(1,2);
            gateLo=nan(1,2); gateHi=nan(1,2);
            if family=="window", gateCount=1; end
            if family=="staggered_windows", gateCount=2; end
            for gate=1:gateCount
                floorBox=data.scene.obstacles(2*gate-1);
                ceilingBox=data.scene.obstacles(2*gate);
                gateLo(gate)=floorBox.zmax+data.q.pathClearance;
                gateHi(gate)=ceilingBox.zmin-data.q.pathClearance;
                gateCenters(gate)=(floorBox.zmax+ceilingBox.zmin)/2;
                assert(gateLo(gate)<gateHi(gate));
                covered=any(data.q.heightLayers>=gateLo(gate)-1e-10 & ...
                    data.q.heightLayers<=gateHi(gate)+1e-10);
                gatesWithoutLayer=gatesWithoutLayer+~covered;
            end
            if gatesWithoutLayer>0 && ~strcmp(data.scene.kind,'generation_failed')
                assert(reason=="planning_failed", ...
                    'UAV3D:AuditGate','seed %d missed gate did not report planning_failed.',seed);
            end
            record=struct('seed',seed,'family',family,'success',success, ...
                'reason',reason,'cycles',cycles,'gateCount',gateCount, ...
                'gatesWithoutFixedLayer',gatesWithoutLayer, ...
                'gate1Center',gateCenters(1),'gate1ClearLo',gateLo(1), ...
                'gate1ClearHi',gateHi(1),'gate2Center',gateCenters(2), ...
                'gate2ClearLo',gateLo(2),'gate2ClearHi',gateHi(2));
            rawRows=[rawRows;record]; %#ok<AGROW>
            seen(end+1)=seed; %#ok<AGROW>
        end
        fprintf('AUDIT_WORKER=%d records=%d\n',worker,numel(seen));
    end
    assert(isequal(sort(seen),1:1000));
    raw=sortrows(struct2table(rawRows),'seed');
    writetable(raw,fullfile(studyDir,'raw_mat_audit.csv'));
    audit=struct('passed',true,'records',height(raw),'uniqueSeeds',numel(unique(raw.seed)), ...
        'success',nnz(raw.success),'failure',nnz(~raw.success), ...
        'allCsvRowsMatchMat',true,'allInitialParametersMatchProtocol',true, ...
        'allFixedLayersUnchanged',true,'allSuccessHoldAndClearanceChecksPass',true, ...
        'missedWindowSeeds',nnz(raw.gatesWithoutFixedLayer>0), ...
        'wallSeconds',toc(auditClock),'completedLocal',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));
    fid=fopen(fullfile(studyDir,'raw_mat_audit.json'),'w','n','UTF-8');
    fprintf(fid,'%s',jsonencode(audit,PrettyPrint=true)); fclose(fid);
    disp(audit);
end

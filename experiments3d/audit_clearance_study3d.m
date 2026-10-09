function audit_clearance_study3d(studyDir)
% AUDIT_CLEARANCE_STUDY3D Re-read every MAT; independently check true states.
    oldPath=path; guard=onCleanup(@() path(oldPath)); %#ok<NASGU>
    snap=fullfile(studyDir,'source_snapshot'); addpath(fullfile(snap,'src3d'));
    saved=load(fullfile(studyDir,'protocol.mat'),'protocol');
    maps=readtable(fullfile(studyDir,'map_manifest.csv'),'Delimiter',',','VariableNamingRule','preserve','TextType','string');
    for mapIndex=1:height(maps)
        map=load(fullfile(studyDir,char(maps.mapFile(mapIndex))),'scene','mapHash');
        bytes=unicode2native(jsonencode(struct('workspace',saved.protocol.generationP.workspace, ...
            'obstacles',map.scene.obstacles)),'UTF-8');
        md=java.security.MessageDigest.getInstance('SHA-256'); md.update(bytes);
        digest=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
        assert(strcmp(digest,maps.mapHash(mapIndex)) && strcmp(digest,map.mapHash), ...
            'UAV3D:MapChanged','Frozen geometry changed for seed %d.',maps.seed(mapIndex));
    end
    fprintf('MAP_GEOMETRY_AUDIT_PASSED maps=%d\n',height(maps));
    files=dir(fullfile(studyDir,'stages','**','trials','*.mat'));
    rows=struct([]); problems=strings(0,1); executionErrors=0;
    for k=1:numel(files)
        filename=fullfile(files(k).folder,files(k).name); x=load(filename);
        try
            q=x.q; row=x.row; r=x.r; j=x.job;
            assert(q.vmax==3 && q.amax==3 && q.obsSafe==0.6 && q.uavSafe==1);
            assert(q.dt==0.05 && q.Tmax==120 && q.safety.enable && q.adp.enable);
            assert(all(q.adp.wA==0,'all') && all(q.adp.wC==0));
            assert(q.pathClearance==j.pathXY && q.desiredClear==j.refXY);
            assert(q.directionalClearance.planningZ==j.planZ && q.directionalClearance.referenceZ==j.refZ);
            assert(q.noiseStd==j.noiseStd && q.disturbanceMax==j.disturbanceMax);
            assert(q.stochasticSeed==j.stochasticSeed && q.seed==j.seed);
            map=load(fullfile(studyDir,char(j.mapFile)),'scene','mapHash');
            assert(isequaln(x.scene,map.scene) && strcmp(x.mapHash,map.mapHash));
            assert(strcmp(row.mapHash,map.mapHash) && row.jobId==j.jobId);
            n=0; minPoint=nan; minPair=nan; blendHeightGap=0; blendReferenceMin=nan; blendChecks=0;
            finalVPairRms=nan; finalVPairMax=nan; referenceVPairRms=nan;
            transitHeightGapMax=nan; terminalHeightGapMax=nan;
            tailSteps=0; tailFormViolationSteps=nan; tailGoalViolationSteps=nan;
            tailSpeedViolationSteps=nan; tailReferenceBlockedSteps=nan;
            if isempty(x.errorText)
                assert(~isempty(r)); assert(row.success==r.success && row.reason==string(r.reason));
            else
                executionErrors=executionErrors+1;
                assert(row.reason=="execution_error" && ~row.success);
            end
            if ~isempty(r) && isfield(r.history,'t') && ~isempty(r.history.t)
                h=r.history; n=numel(h.t); assert(n==row.cycles);
                points=reshape(h.state(1:3,:,:),3,[]);
                minPoint=min(directional_clearance3d(points,x.scene.obstacles,[1;1;1]));
                minPair=inf;
                for a=1:q.nUAV-1
                    for b=a+1:q.nUAV
                        difference=reshape(h.state(1:3,a,:)-h.state(1:3,b,:),3,[]);
                        minPair=min(minPair,min(vecnorm(difference)));
                    end
                end
                assert(abs(row.minObstacleClearance-r.metrics.minObstacleClearance)<1e-9);
                assert(abs(row.minPairDistance-r.metrics.minPairDistance)<1e-9);
                assert(minPoint>=row.minObstacleClearance-2e-6 && minPair>=row.minPairDistance-2e-6);
                assert(abs(row.path-r.metrics.leaderPathLength)<1e-9);
                assert(abs(row.verticalTravel-r.metrics.leaderVerticalTravel)<1e-9);
                tailIds=max(1,n-ceil(q.holdDuration/q.dt)+1):n;
                tailSteps=numel(tailIds);
                tailFormViolationSteps=nnz(h.formationError(tailIds)>q.formTol+1e-10);
                tailLeader=reshape(h.state(1:3,1,tailIds),3,[]);
                tailGoalViolationSteps=nnz(vecnorm(tailLeader-q.target)>q.goalTol+1e-10);
                tailSpeed=reshape(vecnorm(reshape(h.state(4:6,:,tailIds),3,[])),q.nUAV,[]);
                tailSpeedViolationSteps=nnz(max(tailSpeed,[],1)>q.vTol+1e-10);
                tailReferenceBlockedSteps=nnz(h.referenceBlocked(tailIds));
                % Pair distances compare actual shape with the original V and
                % avoid needing a stored yaw angle. Reference error is distinct.
                offsets=zeros(2,q.nUAV);
                for a=2:q.nUAV
                    level=ceil((a-1)/2); side=2*mod(a,2)-1;
                    offsets(:,a)=[-level*q.formSpacing;side*level*q.formWing];
                end
                pairError=[]; referencePairError=[];
                for a=1:q.nUAV-1
                    for b=a+1:q.nUAV
                        expected=norm(offsets(:,a)-offsets(:,b));
                        pairError(end+1)=norm(r.state(1:3,a)-r.state(1:3,b))-expected; %#ok<AGROW>
                        referencePairError(end+1)=norm(h.reference(:,a,end)-h.reference(:,b,end))-expected; %#ok<AGROW>
                    end
                end
                finalVPairRms=sqrt(mean(pairError.^2)); finalVPairMax=max(abs(pairError));
                referenceVPairRms=sqrt(mean(referencePairError.^2));
                heightGap=max(abs(reshape(h.state(3,2:end,:)-h.state(3,1,:),q.nUAV-1,[])),[],1);
                firstBlend=find(h.holdAlpha>1e-9,1);
                if isempty(firstBlend)
                    transitHeightGapMax=max(heightGap);
                else
                    terminalStart=max(1,firstBlend-2);
                    transitHeightGapMax=0;
                    if terminalStart>1, transitHeightGapMax=max(heightGap(1:terminalStart-1)); end
                    terminalHeightGapMax=max(heightGap(terminalStart:end));
                end
                % The frozen reference projector uses the common leader-height
                % slice. Terminal blending can temporarily give references a
                % different height; audit their actual inflated-solid clearance.
                if q.directionalClearance.enable
                    refs=reshape(h.reference(:,2:end,:),3,q.nUAV-1,[]);
                    delta=reshape(refs(3,:,:)-h.reference(3,1,:),q.nUAV-1,[]);
                    blendHeightGap=max(abs(delta),[],'all'); mask=abs(delta)>1e-10;
                    if any(mask,'all')
                        refPoints=reshape(refs,3,[]); refPoints=refPoints(:,mask(:));
                        margins=[q.desiredClear;q.desiredClear;q.directionalClearance.referenceZ];
                        blendReferenceMin=min(directional_clearance3d(refPoints,x.scene.obstacles,margins));
                        blendChecks=size(refPoints,2);
                    end
                end
                if r.success
                    assert(row.minObstacleClearance>q.obsSafe && row.minPairDistance>q.uavSafe);
                    assert(minPoint>q.obsSafe && minPair>q.uavSafe);
                    holdSteps=ceil(q.holdDuration/q.dt); assert(n>=holdSteps);
                    ids=n-holdSteps+1:n;
                    assert(all(h.formationError(ids)<=q.formTol+1e-10));
                    speeds=vecnorm(reshape(h.state(4:6,:,ids),3,[]));
                    assert(all(speeds<=q.vTol+1e-10));
                    leader=reshape(h.state(1:3,1,ids),3,[]);
                    assert(all(vecnorm(leader-q.target)<=q.goalTol+1e-10));
                    assert(~any(h.referenceBlocked(ids)));
                    assert(norm(r.state(1:3,1)-q.target)<=q.goalTol+1e-10);
                end
            else
                assert(row.cycles==0 && isnan(row.path) && isnan(row.verticalTravel));
            end
            rows=[rows;struct('jobId',row.jobId,'stage',row.stage,'configId',row.configId, ...
                'seed',row.seed,'mapHash',row.mapHash,'reason',row.reason, ...
                'success',row.success,'cycles',n,'minVertexObstacle',minPoint,'minVertexPair',minPair, ...
                'blendHeightGap',blendHeightGap,'blendReferenceMin',blendReferenceMin,'blendChecks',blendChecks, ...
                'finalVPairRms',finalVPairRms,'finalVPairMax',finalVPairMax, ...
                'referenceVPairRms',referenceVPairRms,'transitHeightGapMax',transitHeightGapMax, ...
                'terminalHeightGapMax',terminalHeightGapMax,'tailSteps',tailSteps, ...
                'tailFormViolationSteps',tailFormViolationSteps,'tailGoalViolationSteps',tailGoalViolationSteps, ...
                'tailSpeedViolationSteps',tailSpeedViolationSteps, ...
                'tailReferenceBlockedSteps',tailReferenceBlockedSteps)]; %#ok<AGROW>
        catch exception
            problems(end+1)=string(filename)+": "+string(exception.message); %#ok<AGROW>
        end
        if mod(k,250)==0, fprintf('RAW_AUDIT_PROGRESS=%d/%d problems=%d\n',k,numel(files),numel(problems)); end
    end
    if ~isempty(rows), writetable(struct2table(rows),fullfile(studyDir,'raw_mat_audit.csv')); end
    result=struct('records',numel(files),'checked',numel(rows),'passed',isempty(problems),'mapGeometryCount',height(maps), ...
        'executionErrors',executionErrors,'problems',problems, ...
        'independentChecks','Map identity, input/row equality, all true state vertices and pair distances; original stored swept-distance records and final hold checks reconciled.', ...
        'limits','Vertex recomputation does not replace the frozen simulator swept checks. No independent full continuous trajectory proof.');
    fid=fopen(fullfile(studyDir,'raw_mat_audit.json'),'w','n','UTF-8');
    fprintf(fid,'%s',jsonencode(result,PrettyPrint=true)); fclose(fid);
    assert(isempty(problems),'UAV3D:RawAudit','Raw MAT audit found %d inconsistencies.',numel(problems));
    fprintf('RAW_MAT_AUDIT_PASSED records=%d executionErrors=%d\n',numel(files),executionErrors);
end

function audit_normal_seed_study3d(studyDir,worker)
% AUDIT_NORMAL_SEED_STUDY3D Audit every raw proposal, frozen map and full-flight MAT.
    snap=fullfile(studyDir,'source_snapshot');
    addpath(fullfile(snap,'config'),fullfile(snap,'src3d'),fullfile(snap,'experiments3d'));
    stored=load(fullfile(studyDir,'protocol.mat'),'protocol');protocol=stored.protocol;
    directory=fullfile(studyDir,'workers',sprintf('worker_%02d',worker));
    opts=detectImportOptions(fullfile(directory,'flight.csv'),'Delimiter',',','VariableNamingRule','preserve');
    names={'mapFile','mapHash','generationReason','witnessStatus','reason','planStatus','fixedPlanStatus','addedHeights'};
    opts=setvartype(opts,names,'string');csv=readtable(fullfile(directory,'flight.csv'),opts);
    range=protocol.ranges(worker,:);assert(isequal(csv.seed',(range(1):range(2))));
    records=struct([]);timer=tic;
    for index=1:height(csv)
        seed=csv.seed(index);mapFile=fullfile(studyDir,'maps',sprintf('seed_%06d.mat',seed));f=load(mapFile);
        rawFile=fullfile(directory,'trials',sprintf('normal_%06d.mat',seed));d=load(rawFile);
        q=d.q;scene=d.scene;r=d.r;rules=protocol.generation;
        assert(isempty(d.errorText) && ~isempty(r));assert(isequaln(q,f.q) && isequaln(scene,f.scene));
        expected=protocol.p;expected.seed=seed;expected.randomObstacleCount=f.metadata.obstacleCount;
        assert(isequaln(q,expected) && all(q.adp.wC==0,'all') && all(q.adp.wA==0,'all'));
        assert(isequaln(scene,r.scene));
        actualParams=r.params;actualParams.adp.wC=q.adp.wC;actualParams.adp.wA=q.adp.wA;assert(isequaln(actualParams,q));
        assert(strcmp(normal_seed_study_io3d('map_hash',q,scene),f.mapHash) && strcmp(f.mapHash,d.mapHash));
        audit=scene.generationAudit;assert(audit.requestedSeed==seed && audit.attempts==numel(audit.proposals));
        stream=RandStream('mt19937ar','Seed',seed);
        for k=1:audit.attempts
            a=audit.proposals{k};proposalSeed=randi(stream,2^32)-1;assert(a.attempt==k && a.proposalSeed==proposalSeed);
            [original,metadata]=random_solids3d(proposalSeed,q,rules.sampling);
            assert(isequaln(original,a.scene) && isequaln(metadata,a.samplingMetadata));
            assert(~a.check.accepted || (k==audit.attempts && audit.accepted));
        end
        assert(isequaln(scene.obstacles,audit.proposals{end}.scene.obstacles));
        assert(audit.rejectedMaps==audit.attempts-double(audit.accepted));
        assert(audit.accepted==f.metadata.accepted && audit.accepted==d.row.accepted);
        independentWitnessClearance=nan;witnessBoundary=nan;
        if audit.accepted
            c=scene.generation.check;assert(c.accepted && strcmp(c.feasibilityStatus,'certified'));
            initial=init_formation3d(q);offsets=initial(1:3,:)-q.start;
            assert(abs(independent_min(initial(1:3,:),scene.obstacles)-c.initialClearance)<1e-9);
            assert(independent_min(initial(1:3,:),scene.obstacles)>=rules.startClearance-1e-9);
            assert(independent_min(q.target+offsets,scene.obstacles)>=rules.terminalClearance-1e-9);
            w=c.witness;assert(isequaln(offsets,w.offsets));assert(norm(w.points(:,1)-q.start)<1e-9 && norm(w.points(:,end)-q.target)<1e-9);
            assert(norm(w.points(:,2)-c.departurePoint)<1e-9 && norm(w.points(:,end-1)-c.arrivalPoint)<1e-9);
            independentWitnessClearance=inf;witnessBoundary=inf;
            for k=1:size(w.points,2)-1
                for vehicle=1:q.nUAV
                    a=w.points(:,k)+offsets(:,vehicle);b=w.points(:,k+1)+offsets(:,vehicle);
                    gap=segment_clearance3d(a,b,scene.obstacles);assert(gap>=rules.witnessClearance-1e-7);
                    independentWitnessClearance=min(independentWitnessClearance,gap);
                    witnessBoundary=min(witnessBoundary,min([a-q.workspace(1,:)',q.workspace(2,:)'-a,b-q.workspace(1,:)',q.workspace(2,:)'-b],[],'all'));
                end
            end
            assert(witnessBoundary>=rules.boundaryClearance-1e-9);
            lengths=vecnorm(diff(w.points,1,2));peaks=min(q.vmax,sqrt(q.amax*lengths));
            durations=lengths./max(peaks,eps)+peaks/q.amax;
            assert(max(abs(w.segmentPeakSpeeds-peaks))<1e-9 && max(abs(w.segmentDurations-durations))<1e-9);
            assert(abs(w.duration-sum(durations)-q.terminalBlendTime-q.holdDuration)<1e-9 && w.duration<=q.Tmax);
        else
            assert(audit.attempts==rules.maxAttempts && strcmp(scene.kind,'generation_failed'));
            assert(~r.success && strcmp(r.reason,'generation_failed'));
        end
        names=csv.Properties.VariableNames;
        for k=1:numel(names)
            name=names{k};a=csv.(name)(index,:);b=d.row.(name);
            if ischar(b) || isstring(b)
                assert((all(ismissing(a)) && strlength(string(b))==0) || strcmp(string(a),string(b)));
            else
                assert((isnan(a)&&isnan(b)) || a==b || abs(a-b)<=3e-12*max(1,abs(b)));
            end
        end
        assert(r.success==d.row.success && strcmp(r.reason,d.row.reason));assert(r.success==strcmp(r.reason,'success'));
        vertexMin=nan;pairMin=nan;maxVelocity=nan;cycles=0;
        if isfield(r.history,'t') && ~isempty(r.history.t)
            h=r.history;cycles=numel(h.t);assert(all(h.noise==0,'all') && all(h.disturbance==0,'all'));
            vertexMin=independent_min(reshape(h.state(1:3,:,:),3,[]),scene.obstacles);pairMin=inf;
            for a=1:q.nUAV-1
                for b=a+1:q.nUAV,delta=squeeze(h.state(1:3,a,:)-h.state(1:3,b,:));pairMin=min(pairMin,min(vecnorm(delta)));end
            end
            assert(vertexMin>=min(h.obstacleClearance)-1e-7 && pairMin>=min(h.pairDistance)-1e-7);
            maxVelocity=max(sqrt(sum(h.state(4:6,:,:).^2,1)),[],'all');assert(maxVelocity<=q.vmax+1e-9);
            assert(r.time<=q.Tmax+1e-9 && abs(r.time-h.t(end))<1e-9);
            assert(isequaln(r.state,h.state(:,:,end)));
            followerError=max(sqrt(sum((h.state(1:3,2:end,:)-h.reference(:,2:end,:)).^2,1)),[],2);
            assert(max(abs(reshape(followerError,[],1)-h.formationError))<1e-9);
            if r.success
                count=ceil(q.holdDuration/q.dt-1e-10);assert(cycles>=count);final=h.state(:,:,end-count+1:end);
                assert(all(vecnorm(squeeze(final(1:3,1,:))-q.target)<=q.goalTol+1e-9));
                assert(all(sqrt(sum(final(4:6,:,:).^2,1))<=q.vTol+1e-9,'all'));
                assert(all(h.formationError(end-count+1:end)<=q.formTol+1e-9) && ~any(h.referenceBlocked(end-count+1:end)));
                assert(min(h.obstacleClearance)>q.obsSafe && min(h.pairDistance)>q.uavSafe);
            end
        else
            assert(~r.success && ismember(string(r.reason),["generation_failed","invalid_scene","planning_failed"]));
        end
        assert(cycles==d.row.cycles);
        records=[records;struct('seed',seed,'accepted',audit.accepted,'attempts',audit.attempts, ...
            'success',r.success,'reason',string(r.reason),'parametersPassed',true,'allProposalsRegenerated',true, ...
            'witnessChecked',audit.accepted,'independentWitnessClearance',independentWitnessClearance, ...
            'witnessBoundaryClearance',witnessBoundary,'csvPassed',true,'cycles',cycles, ...
            'vertexMin',vertexMin,'pairVertexMin',pairMin,'maxVelocity',maxVelocity,'successHoldChecked',r.success, ...
            'mapMatSha256',string(midpoint_sha256_file3d(mapFile)),'rawMatSha256',string(midpoint_sha256_file3d(rawFile)))]; %#ok<AGROW>
        if mod(index,10)==0 || index==height(csv)
            normal_seed_study_io3d('table',fullfile(directory,'audit.csv'),struct2table(records));
            normal_seed_study_io3d('json',fullfile(directory,'audit_status.json'),struct('phase','audit','worker',worker, ...
                'status','running','completed',index,'success',nnz([records.success]),'wallSeconds',toc(timer)));
            fprintf('NORMAL_AUDIT worker=%d checked=%d/%d\n',worker,index,height(csv));
        end
    end
    normal_seed_study_io3d('json',fullfile(directory,'audit_status.json'),struct('phase','audit','worker',worker, ...
        'status','complete','passed',true,'completed',height(csv),'success',nnz([records.success]),'wallSeconds',toc(timer)));
end
function value=independent_min(points,obstacles)
    value=inf;
    for o=obstacles
        if strcmp(o.type,'sphere')
            ds=vecnorm(points-o.center)-o.r;
        elseif strcmp(o.type,'cylinder')
            radial=vecnorm(points(1:2,:)-o.center)-o.r;vertical=max(o.zmin-points(3,:),points(3,:)-o.zmax);
            ds=sqrt(max(radial,0).^2+max(vertical,0).^2)+min(max(radial,vertical),0);
        else
            offsets=abs(points-o.center)-o.halfSize;ds=sqrt(sum(max(offsets,0).^2,1))+min(max(offsets,[],1),0);
        end
        value=min(value,min(ds));
    end
end

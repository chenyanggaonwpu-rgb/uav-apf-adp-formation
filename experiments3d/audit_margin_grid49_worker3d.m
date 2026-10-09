function audit_margin_grid49_worker3d(studyDir,workerIndex)
% AUDIT_MARGIN_GRID49_WORKER3D Check every saved task and exact baseline replay.
    snap=fullfile(studyDir,'source_snapshot');
    addpath(fullfile(snap,'config'),fullfile(snap,'src3d'),fullfile(snap,'experiments3d'));
    protocol=jsondecode(fileread(fullfile(studyDir,'protocol.json')));
    manifest=readtable(fullfile(studyDir,'map_manifest.csv'),'TextType','string');
    directory=fullfile(studyDir,'workers',sprintf('worker_%02d',workerIndex));
    opts=detectImportOptions(fullfile(directory,'trials.csv'),'Delimiter',',','VariableNamingRule','preserve');
    opts=setvartype(opts,{'jobId','stage','configId','family','mapHash','reason','planStatus','layerPolicy','inputMapSha256'},'string');
    csv=readtable(fullfile(directory,'trials.csv'),opts);records=struct([]);baseChecked=0;clock=tic;
    for i=1:height(csv)
        d=load(fullfile(directory,'trials',char(csv.jobId(i)+".mat")));
        m=manifest(manifest.seed==d.row.seed,:);f=load(fullfile(studyDir,char(m.mapFile)));
        assert(isequaln(d.scene,f.scene) && strcmp(d.mapHash,m.mapHash));
        expected=f.q;expected.pathClearance=d.job.pathXY;expected.desiredClear=d.job.refXY;
        expected.directionalClearance=struct('enable',false,'planningZ',d.job.pathXY,'referenceZ',d.job.refXY);
        expected.stochasticSeed=d.job.seed;expected.adp.wA=zeros(3,6);expected.adp.wC=zeros(6,1);
        assert(isequaln(d.q,expected) && isempty(d.errorText) && ~isempty(d.r));
        assert(d.q.obsSafe==.6 && d.q.uavSafe==1 && d.q.noiseStd==0 && d.q.disturbanceMax==0);
        assert(d.q.adp.enable && d.q.safety.enable && d.q.adaptiveHeight.enable);
        assert(isequal(d.q.heightLayers,[5 10 15 20 25]));
        for j=1:numel(csv.Properties.VariableNames)
            name=csv.Properties.VariableNames{j};a=csv.(name)(i,:);b=d.row.(name);
            if ischar(b)||isstring(b),assert(strcmp(string(a),string(b)));
            else,assert((isnan(a)&&isnan(b)) || a==b || abs(a-b)<=2e-12*max(1,abs(b)));end
        end
        r=d.r;q=d.q;assert(r.success==d.row.success && strcmp(r.reason,d.row.reason));
        assert(r.success==strcmp(r.reason,'success'));
        assert(strcmp(r.reason,'invalid_scene')==~logical(m.initialHardSafe));
        routeMin=nan;
        if isfield(r,'initialRoute') && ~isempty(r.initialRoute)
            points=[q.start,r.initialRoute];routeMin=inf;
            for j=1:size(points,2)-1
                routeMin=min(routeMin,segment_clearance3d(points(:,j),points(:,j+1),d.scene.obstacles));
            end
            assert(routeMin>=q.pathClearance-1e-6);
        end
        vertexMin=nan;pairMin=nan;maxSpeed=nan;cycles=0;
        if isfield(r.history,'t') && ~isempty(r.history.t)
            h=r.history;cycles=numel(h.t);
            assert(all(h.noise==0,'all') && all(h.disturbance==0,'all'));
            vertexMin=independent_min(reshape(h.state(1:3,:,:),3,[]),d.scene.obstacles);pairMin=inf;
            for a=1:q.nUAV-1
                for b=a+1:q.nUAV
                    pairMin=min(pairMin,min(vecnorm(squeeze(h.state(1:3,a,:)-h.state(1:3,b,:)))));
                end
            end
            assert(vertexMin>=min(h.obstacleClearance)-1e-7 && pairMin>=min(h.pairDistance)-1e-7);
            maxSpeed=max(sqrt(sum(h.state(4:6,:,:).^2,1)),[],'all');assert(maxSpeed<=q.vmax+1e-9);
            if r.success
                count=ceil(q.holdDuration/q.dt-1e-10);final=h.state(:,:,end-count+1:end);
                assert(all(vecnorm(squeeze(final(1:3,1,:))-q.target)<=q.goalTol+1e-9));
                assert(all(sqrt(sum(final(4:6,:,:).^2,1))<=q.vTol+1e-9,'all'));
                assert(all(h.formationError(end-count+1:end)<=q.formTol+1e-9));
                assert(~any(h.referenceBlocked(end-count+1:end)));
                assert(min(h.obstacleClearance)>q.obsSafe && min(h.pairDistance)>q.uavSafe);
                assert(vertexMin>q.obsSafe && pairMin>q.uavSafe);
            end
        end
        assert(cycles==d.row.cycles);
        baseline=false;
        if q.pathClearance==4.2 && q.desiredClear==4
            oldWorker=ceil((q.seed-2000)/50);
            old=load(fullfile(protocol.baseStudy,'workers',sprintf('worker_%02d',oldWorker), ...
                'trials',sprintf('random_%06d.mat',q.seed)),'r');
            for field={'success','reason','time','state','route'}
                assert(isequaln(r.(field{1}),old.r.(field{1})),field{1});
            end
            current=r.history;prior=old.r.history;
            if isfield(current,'cycleMs'),current=rmfield(current,'cycleMs');prior=rmfield(prior,'cycleMs');end
            assert(isequaln(current,prior),'UAV3D:BaselineChanged','Baseline replay changed in seed %d.',q.seed);
            if isfield(r,'metrics'),assert(isequaln(r.metrics,old.r.metrics));end
            baseline=true;baseChecked=baseChecked+1;
        end
        records=[records;struct('jobId',d.row.jobId,'configId',d.row.configId,'seed',q.seed, ...
            'success',r.success,'reason',string(r.reason),'parametersPassed',true,'csvPassed',true, ...
            'mapPassed',true,'initialRouteClearance',routeMin,'independentVertexMin',vertexMin, ...
            'independentPairMin',pairMin,'maxSpeed',maxSpeed,'baselineExactReplayChecked',baseline, ...
            'rawMatSha256',string(midpoint_sha256_file3d(fullfile(directory,'trials',char(d.row.jobId+".mat")))) )]; %#ok<AGROW>
        if mod(i,100)==0,fprintf('MARGIN_GRID_AUDIT worker=%d records=%d/%d\n',workerIndex,i,height(csv));end
    end
    writetable(struct2table(records),fullfile(directory,'raw_audit.csv'));
    value=struct('passed',true,'worker',workerIndex,'records',height(csv), ...
        'baselineExactReplays',baseChecked,'success',nnz([records.success]),'wallSeconds',toc(clock));
    fid=fopen(fullfile(directory,'audit.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
function value=independent_min(points,obstacles)
    value=inf;
    for o=obstacles
        if strcmp(o.type,'sphere'),ds=vecnorm(points-o.center)-o.r;
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

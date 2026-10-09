function audit_midpoint_sweep_repair01_3d(studyDir)
% AUDIT_MIDPOINT_SWEEP3D Raw-MAT audit with independent gate and vertex checks.
    snap=fullfile(studyDir,'source_snapshot');
    addpath(fullfile(snap,'config'),fullfile(snap,'src3d'),fullfile(snap,'experiments3d'));
    saved=load(fullfile(studyDir,'protocol.mat'),'protocol'); protocol=saved.protocol;
    manifest=readtable(fullfile(studyDir,'map_manifest.csv'),'Delimiter',',','TextType','string','VariableNamingRule','preserve');
    hashes=readtable(fullfile(studyDir,'source_sha256.csv'),'Delimiter',',','TextType','string','VariableNamingRule','preserve');
    for i=1:height(hashes)
        assert(strcmp(midpoint_sha256_file3d(fullfile(snap,char(hashes.file(i)))),hashes.sha256(i)));
    end
    records=struct([]); seen=[]; timer=tic;
    for worker=1:protocol.nWorkers
        directory=fullfile(studyDir,'workers',sprintf('worker_%02d',worker));
        status=jsondecode(fileread(fullfile(directory,'status.json'))); assert(strcmp(status.status,'complete'));
        options=detectImportOptions(fullfile(directory,'trials.csv'),'Delimiter',',','VariableNamingRule','preserve');
        options=setvartype(options,{'family','baselineReason','mapHash','reason','planStatus','fixedPlanStatus','addedHeights'},'string');
        csv=readtable(fullfile(directory,'trials.csv'),options);
        assert(height(csv)==250);
        for i=1:height(csv)
            seed=csv.seed(i); seen(end+1)=seed; %#ok<AGROW>
            file=fullfile(directory,'trials',sprintf('adaptive_%06d.mat',seed)); d=load(file);
            m=manifest(manifest.seed==seed,:); assert(height(m)==1);
            expected=d.originalQ; expected.version=protocol.version; expected.adaptiveHeight=protocol.p.adaptiveHeight;
            assert(isequaln(d.q,expected)); assert(isequal(d.q.heightLayers,[5 10 15 20 25]));
            assert(d.q.pathClearance==4.2 && d.q.desiredClear==4.0 && d.q.noiseStd==0 && d.q.disturbanceMax==0);
            assert(isempty(d.errorText) && ~isempty(d.r),'UAV3D:TrialError','seed %d has an execution error.',seed);
            bytes=unicode2native(jsonencode(struct('workspace',d.q.workspace,'obstacles',d.scene.obstacles)),'UTF-8');
            md=java.security.MessageDigest.getInstance('SHA-256'); md.update(bytes);
            actualHash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
            assert(strcmp(actualHash,m.mapHash) && strcmp(actualHash,d.mapHash));
            assert(strcmp(midpoint_sha256_file3d(char(m.baselineMat)),m.baselineMatSha256));
            names=csv.Properties.VariableNames;
            for j=1:numel(names)
                name=names{j}; a=csv.(name)(i,:); b=d.row.(name);
                if ischar(b) || isstring(b)
                    assert((all(ismissing(a)) && strlength(string(b))==0) || strcmp(string(a),string(b)));
                else
                    assert((isnan(a)&&isnan(b)) || a==b || abs(a-b)<=2e-12*max(1,abs(b)));
                end
            end
            r=d.r; assert(r.success==d.row.success && strcmp(r.reason,d.row.reason));
            assert(r.success==strcmp(r.reason,'success'));
            routeClearance=nan;
            if ~isempty(r.initialRoute)
                points=[d.q.start,r.initialRoute]; routeClearance=inf;
                for edge=1:size(points,2)-1
                    routeClearance=min(routeClearance,segment_clearance3d(points(:,edge),points(:,edge+1),d.scene.obstacles));
                end
                assert(routeClearance>=d.q.pathClearance-1e-6);
            end
            hs=r.plan.heightSelection;
            assert(numel(hs.addedHeights)<=protocol.p.adaptiveHeight.maxAddedLayers);
            for j=find(hs.decisions=="added_midpoint")'
                centre=mean(hs.sections(j,3:4));
                assert(any(abs(hs.addedHeights-centre)<1e-8));
                assert(~any(hs.baseLayers>=hs.sections(j,3)-1e-7 & hs.baseLayers<=hs.sections(j,4)+1e-7));
            end
            % Independently recover the original structural gate intervals.
            gates=0; uncovered=0;
            if strcmp(d.scene.family,'window'), gates=1; end
            if strcmp(d.scene.family,'staggered_windows'), gates=2; end
            for g=1:gates
                gateLower=d.scene.obstacles(2*g-1).zmax+d.q.pathClearance;
                gateUpper=d.scene.obstacles(2*g).zmin-d.q.pathClearance;
                covered=any(hs.layers>=gateLower-1e-7 & hs.layers<=gateUpper+1e-7);
                uncovered=uncovered+~covered;
            end
            vertexMin=nan; pairMin=nan; cycles=0;
            if isfield(r.history,'t') && ~isempty(r.history.t)
                h=r.history; cycles=numel(h.t);
                assert(all(h.noise==0,'all') && all(h.disturbance==0,'all'));
                pts=reshape(h.state(1:3,:,:),3,[]); vertexMin=inf;
                for o=d.scene.obstacles
                    if strcmp(o.type,'sphere')
                        ds=vecnorm(pts-o.center)-o.r;
                    elseif strcmp(o.type,'cylinder')
                        radial=vecnorm(pts(1:2,:)-o.center)-o.r;
                        vertical=max(o.zmin-pts(3,:),pts(3,:)-o.zmax);
                        ds=sqrt(max(radial,0).^2+max(vertical,0).^2)+min(max(radial,vertical),0);
                    else
                        offsets=abs(pts-o.center)-o.halfSize;
                        ds=sqrt(sum(max(offsets,0).^2,1))+min(max(offsets,[],1),0);
                    end
                    vertexMin=min(vertexMin,min(ds));
                end
                pairMin=inf;
                for a=1:4
                    for b=a+1:5
                        delta=squeeze(h.state(1:3,a,:)-h.state(1:3,b,:));
                        pairMin=min(pairMin,min(vecnorm(delta)));
                    end
                end
                assert(vertexMin>=min(h.obstacleClearance)-1e-7 && pairMin>=min(h.pairDistance)-1e-7);
                if r.success
                    count=ceil(d.q.holdDuration/d.q.dt-1e-10); final=h.state(:,:,end-count+1:end);
                    assert(all(vecnorm(squeeze(final(1:3,1,:))-d.q.target)<=d.q.goalTol+1e-9));
                    assert(all(sqrt(sum(final(4:6,:,:).^2,1))<=d.q.vTol+1e-9,'all'));
                    assert(all(h.formationError(end-count+1:end)<=d.q.formTol+1e-9));
                    assert(~any(h.referenceBlocked(end-count+1:end)));
                    assert(min(h.obstacleClearance)>d.q.obsSafe && min(h.pairDistance)>d.q.uavSafe);
                    assert(vertexMin>d.q.obsSafe && pairMin>d.q.uavSafe && uncovered==0);
                end
            end
            assert(cycles==d.row.cycles);
            record=struct('seed',seed,'success',r.success,'reason',string(r.reason), ...
                'mapHashPassed',true,'parametersPassed',true,'csvPassed',true,'successHoldChecked',r.success,'successHoldPassed',r.success, ...
                'initialRouteClearance',routeClearance,'gateCount',gates,'uncoveredGateCount',uncovered,'vertexMin',vertexMin,'pairVertexMin',pairMin, ...
                'rawMatSha256',string(midpoint_sha256_file3d(file)));
            records=[records;record]; %#ok<AGROW>
        end
        fprintf('MIDPOINT_AUDIT worker=%d checked=%d\n',worker,numel(records));
    end
    assert(isequal(sort(seen),1:1000));
    raw=sortrows(struct2table(records),'seed'); writetable(raw,fullfile(studyDir,'raw_mat_audit.csv'));
    audit=struct('passed',true,'records',height(raw),'uniqueSeeds',numel(unique(raw.seed)), ...
        'success',nnz(raw.success),'failure',nnz(~raw.success),'sourceFiles',height(hashes), ...
        'allOriginalMapHashesPassed',true,'allOriginalBaselinesUnchanged',true, ...
        'independentVertexAndSuccessHoldChecksPassed',true,'wallSeconds',toc(timer));
    fid=fopen(fullfile(studyDir,'raw_mat_audit.json'),'w','n','UTF-8'); fprintf(fid,'%s',jsonencode(audit,PrettyPrint=true)); fclose(fid);
    fprintf('MIDPOINT_AUDIT_PASSED records=%d success=%d\n',height(raw),nnz(raw.success));
end

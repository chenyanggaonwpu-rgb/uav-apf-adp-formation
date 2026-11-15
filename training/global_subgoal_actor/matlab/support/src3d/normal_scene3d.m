function [scene,metadata,audit]=normal_scene3d(seed,p,options)
% NORMAL_SCENE3D Bounded whole-map rejection based only on explicit environment checks.
% Every proposal (including rejected geometry) is returned for preservation before flight.
    if nargin<3
        if isfield(p,'normalScene'), options=p.normalScene; else, options=struct(); end
    end
    rules=validate_normal_scene_options3d(p,options);
    validateattributes(seed,{'numeric'},{'scalar','real','finite','integer','nonnegative','<=',2^32-1});
    empty=struct('type',{},'center',{},'r',{},'zmin',{},'zmax',{},'halfSize',{});
    probe=struct('obstacles',empty); localRules=rules; localRules.requireWitness=false;
    baseline=check_normal_scene3d(probe,p,localRules);
    assert(baseline.accepted,'UAV3D:NormalDepartureGeometry', ...
        'The specified initial/departure/terminal formation does not fit the workspace safely.');
    stream=RandStream('mt19937ar','Seed',seed); proposals=cell(0,1); accepted=false;
    for attempt=1:rules.maxAttempts
        proposalSeed=randi(stream,2^32)-1;
        [candidate,rawMetadata]=random_solids3d(proposalSeed,p,rules.sampling);
        check=check_normal_scene3d(candidate,p,rules);
        proposals{end+1,1}=struct('attempt',attempt,'proposalSeed',proposalSeed, ...
            'scene',candidate,'samplingMetadata',rawMetadata,'check',check); %#ok<AGROW>
        if check.accepted, accepted=true; break; end
    end
    audit=struct('protocol','normal_random_v1','requestedSeed',seed,'options',rules, ...
        'proposals',{proposals},'accepted',accepted,'attempts',attempt, ...
        'rejectedMaps',attempt-double(accepted),'acceptedProposalSeed',proposalSeed, ...
        'selectionRule','Whole-map geometry and independent sufficient witness; no evaluated flight-success filtering.');
    if ~accepted, audit.acceptedProposalSeed=nan; end
    scene=candidate; scene.id='random_normal'; scene.seed=seed; scene.family='normal_unstructured';
    scene.description='Conditioned random solids: five-UAV initial/departure/terminal protection and independent 3-D witness; overlaps allowed.';
    if accepted, scene.kind='random'; else, scene.kind='generation_failed'; end
    metadata=rawMetadata; metadata.protocol=audit.protocol; metadata.accepted=accepted;
    metadata.attempts=attempt; metadata.rejectedMaps=audit.rejectedMaps;
    metadata.proposalSeed=proposalSeed; metadata.check=check;
    metadata.estimatedUnionVolumeFraction=nan;
    if accepted
        lo=p.workspace(1,:)'; hi=p.workspace(2,:)'; step=rules.occupancyResolution;
        [x,y,z]=ndgrid(lo(1)+step/2:step:hi(1),lo(2)+step/2:step:hi(2),lo(3)+step/2:step:hi(3));
        metadata.estimatedUnionVolumeFraction=mean(point_clearance_batch3d([x(:)';y(:)';z(:)'],scene.obstacles)<=0);
    end
    metadata.occupancyMethod='Uniform cell-centre samples; estimate of obstacle union, not exact volume.';
    metadata.occupancyResolution=rules.occupancyResolution;
    scene.generation=metadata; scene.generationAudit=audit;
end

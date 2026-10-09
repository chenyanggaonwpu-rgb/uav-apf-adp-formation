function rules=validate_normal_scene_options3d(p,options)
% VALIDATE_NORMAL_SCENE_OPTIONS3D Merge a bounded, explicit environment protocol.
    rules=default_normal_scene_options3d();
    if nargin<2 || isempty(options), options=struct(); end
    assert(isstruct(options) && isscalar(options),'UAV3D:NormalOptions','Options must be a scalar struct.');
    names=fieldnames(options);
    for k=1:numel(names)
        name=names{k};
        assert(isfield(rules,name),'UAV3D:NormalOptions','Unknown normal-scene option: %s',name);
        if strcmp(name,'sampling')
            sampleNames=fieldnames(options.sampling);
            for j=1:numel(sampleNames)
                key=sampleNames{j};
                assert(isfield(rules.sampling,key),'UAV3D:NormalOptions','Unknown sampling option: %s',key);
                rules.sampling.(key)=options.sampling.(key);
            end
        else
            rules.(name)=options.(name);
        end
    end
    positive={'startClearance','departureLength','departureClearance','terminalClearance', ...
        'boundaryClearance','endpointClearance','witnessClearance','witnessGridResolution', ...
        'certificateTolerance','occupancyResolution'};
    for k=1:numel(positive)
        validateattributes(rules.(positive{k}),{'numeric'},{'scalar','real','finite','positive'},mfilename,positive{k});
    end
    validateattributes(rules.maxAttempts,{'numeric'},{'scalar','integer','positive','<=',10000});
    validateattributes(rules.maxExpandedNodes,{'numeric'},{'scalar','integer','positive','<=',1000000});
    validateattributes(rules.requireWitness,{'logical','numeric'},{'scalar','binary'});
    assert(min([rules.startClearance,rules.departureClearance,rules.terminalClearance, ...
        rules.endpointClearance,rules.witnessClearance])>p.obsSafe, ...
        'UAV3D:NormalOptions','Environment clearances must exceed the hard collision threshold.');
    assert(rules.certificateTolerance<=1e-3 && rules.witnessGridResolution>=0.5 && ...
        rules.occupancyResolution>=0.5,'UAV3D:NormalOptions','Unsupported certificate or grid resolution.');
    names=fieldnames(rules.sampling);
    for k=1:numel(names)
        value=rules.sampling.(names{k});
        validateattributes(value,{'numeric'},{'size',[1 2],'real','finite','positive'});
        assert(value(2)>=value(1),'UAV3D:NormalOptions','Sampling ranges must be ascending.');
    end
    validateattributes(rules.sampling.countRange,{'numeric'},{'integer','<=',200});
    span=p.workspace(2,:)-p.workspace(1,:); s=rules.sampling;
    assert(2*s.sphereRadius(2)<=min(span) && ...
        2*s.cylinderRadius(2)<=min(span(1:2)) && s.cylinderHeight(2)<=span(3) && ...
        all([s.boxWidth(2),s.boxDepth(2),s.boxHeight(2)]<=span), ...
        'UAV3D:SizeRange','The declared random solid sizes must fit in the workspace.');
end

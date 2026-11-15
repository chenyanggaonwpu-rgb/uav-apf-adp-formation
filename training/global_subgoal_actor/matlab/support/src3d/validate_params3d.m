function p = validate_params3d(p)
% VALIDATE_PARAMS3D Reject configurations that cannot represent the 3-D model.
    if ~isfield(p,'verticalCostWeight'), p.verticalCostWeight=0; end
    if ~isfield(p,'planning'),p.planning=struct('prebuildGraphs',true);end
    if ~isfield(p.planning,'prebuildGraphs'),p.planning.prebuildGraphs=true;end
    validateattributes(p.planning.prebuildGraphs,{'logical','numeric'},{'scalar','binary'});
    if ~isfield(p,'adaptiveHeight')
        p.adaptiveHeight=struct('enable',false,'maxAddedLayers',8,'intervalTolerance',1e-7);
    end
    validateattributes(p.adaptiveHeight.enable,{'logical','numeric'},{'scalar','binary'});
    validateattributes(p.adaptiveHeight.maxAddedLayers,{'numeric'},{'scalar','integer','positive','<=',32});
    validateattributes(p.adaptiveHeight.intervalTolerance,{'numeric'},{'scalar','finite','positive','<=',1e-3});
    if ~isfield(p,'directionalClearance')
        p.directionalClearance=struct('enable',false, ...
            'planningZ',p.pathClearance,'referenceZ',p.desiredClear);
    end
    if p.directionalClearance.enable
        values=[p.pathClearance,p.desiredClear, ...
            p.directionalClearance.planningZ,p.directionalClearance.referenceZ];
        if any(~isfinite(values)) || any(values<p.obsSafe)
            error('UAV3D:DirectionalMargin','All directional margins must be at least obsSafe.');
        end
    end
    if ~isscalar(p.verticalCostWeight) || ~isfinite(p.verticalCostWeight) || ...
            p.verticalCostWeight<0
        error('UAV3D:InvalidParameter','verticalCostWeight must be finite and nonnegative.');
    end
    if ~isequal(size(p.workspace),[2 3]) || any(~isfinite(p.workspace),'all') || ...
            any(p.workspace(2,:) <= p.workspace(1,:)) || ...
            ~isequal(size(p.start),[3 1]) || ~isequal(size(p.target),[3 1]) || ...
            any(~isfinite([p.start;p.target]))
        error('UAV3D:InvalidParameter','Three-dimensional workspace/start/target required.');
    end
    if any(p.start <= p.workspace(1,:)') || any(p.start >= p.workspace(2,:)') || ...
            any(p.target <= p.workspace(1,:)') || any(p.target >= p.workspace(2,:)')
        error('UAV3D:InvalidParameter','Start and target must be inside the workspace.');
    end
    positive = {'dt','Tmax','vmax','amax','pathGridResolution','goalTol', ...
        'formTol','uavSafe','rho0','waypointTol','replanAfter','terminalBlendTime'};
    for i=1:numel(positive)
        x=p.(positive{i});
        if ~isscalar(x) || ~isfinite(x) || x<=0
            error('UAV3D:InvalidParameter','Invalid positive parameter: %s',positive{i});
        end
    end
    if p.vmax ~= 3 || p.amax ~= 3
        error('UAV3D:InvalidParameter','The project baseline requires 3 m/s and 3 m/s^2.');
    end
    if p.pathClearance < p.obsSafe || p.noiseStd<0 || p.disturbanceMax<0 || ...
            ~isscalar(p.disturbanceHoldSteps) || p.disturbanceHoldSteps<1 || ...
            p.disturbanceHoldSteps~=fix(p.disturbanceHoldSteps)
        error('UAV3D:InvalidParameter','Invalid safety or disturbance settings.');
    end
    if ~isvector(p.heightLayers) || numel(p.heightLayers)<2 || ...
            any(diff(p.heightLayers)<=0) || ...
            any(p.heightLayers<=p.workspace(1,3)) || ...
            any(p.heightLayers>=p.workspace(2,3)) || ...
            any(~isfinite(p.heightLayers))
        error('UAV3D:InvalidParameter','Invalid height layers.');
    end
    if ~isequal(size(p.adp.wC),[6 1]) || ~isequal(size(p.adp.wA),[3 6]) || ...
            any(~isfinite(p.adp.wC)) || any(~isfinite(p.adp.wA),'all')
        error('UAV3D:InvalidParameter','3-D ADP requires wC=6x1 and wA=3x6.');
    end
    if isfield(p.adp,'mode')
        if ~ismember(p.adp.mode,{'legacy6','tracking_value_v1','tracking_value_v2','tracking_value_v3','tracking_value_v4'})
            error('UAV3D:InvalidParameter','Unknown ADP mode.');
        end
        if ismember(p.adp.mode,{'tracking_value_v1','tracking_value_v2','tracking_value_v3','tracking_value_v4'})
            positiveAdp={'gain','uMax','eps','qCtrl','lrA','lrC','maxWeightNorm', ...
                'actorFeatureFloor','criticFeatureFloor'};
            for j=1:numel(positiveAdp)
                value=p.adp.(positiveAdp{j});
                validateattributes(value,{'numeric'},{'scalar','finite','positive'});
            end
            validateattributes(p.adp.gamma,{'numeric'},{'scalar','finite','>',0,'<',1});
            validateattributes(p.adp.lrA,{'numeric'},{'<=',1});
            validateattributes(p.adp.lrC,{'numeric'},{'<=',1});
            validateattributes([p.adp.qForm,p.adp.qVelocity,p.adp.qRisk], ...
                {'numeric'},{'vector','finite','nonnegative'});
            if ismember(p.adp.mode,{'tracking_value_v2','tracking_value_v3','tracking_value_v4'})
                validateattributes(p.adp.residualBaseFraction,{'numeric'}, ...
                    {'scalar','finite','>',0,'<',1});
            end
            if strcmp(p.adp.mode,'tracking_value_v4')
                validateattributes(p.adp.controlLeader,{'logical'},{'scalar'});
            end
            if ismember(p.adp.mode,{'tracking_value_v3','tracking_value_v4'})
                for columns={1:3,4:6}
                    block=p.adp.wA(:,columns{1});
                    if norm(block-block','fro')>1e-8 || min(eig(.5*(block+block')))<-1e-10
                        error('UAV3D:InvalidParameter','Constrained Actor blocks must be symmetric PSD.');
                    end
                end
            end
            if any(p.adp.wC(4:6)<0) || ...
                    min(eig([p.adp.wC(1),p.adp.wC(3);p.adp.wC(3),p.adp.wC(2)]))<-1e-10
                error('UAV3D:InvalidParameter','Tracking value must be PSD with nonnegative risk weights.');
            end
        end
    end
    if p.nUAV~=5 || p.leaderIdx~=1 || ~ismember(p.safety.maxIterations,1:5)
        error('UAV3D:InvalidParameter','This baseline requires five UAVs and at most five safety passes.');
    end
    if ~isscalar(p.randomObstacleCount) || p.randomObstacleCount<0 || ...
            p.randomObstacleCount~=fix(p.randomObstacleCount)
        error('UAV3D:InvalidParameter','randomObstacleCount must be nonnegative integer.');
    end
end

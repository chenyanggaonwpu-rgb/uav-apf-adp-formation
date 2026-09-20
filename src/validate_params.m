function params = validate_params(params)
% VALIDATE_PARAMS 检查仿真参数，尽早报告无效配置。

    required = {'softwareName','version','dt','Tmax','nUAV','leaderIdx','mass','vmax','amax', ...
        'workspace','start','target','formSpacing','formWing','kForm','kFormD', ...
        'formGainBoost','formDampBoost','formErrScale','formTol','vTol', ...
        'holdDuration','navGain','navAccel','closeBoostDist','closeBoostGain', ...
        'kRep','rho0','obsSafe','desiredClear','uavSafe','uavAvoidSafe', ...
        'kUAV','kUAVVel','collisionAvoidGain','collisionVelGain', ...
        'collisionMargin','useTangential','kTang','goalDecay','minGoalRepFactor', ...
        'goalTol','holdTriggerDist','goalScale','escapeThresh','escapeWindow', ...
        'escapeFlipTime','escapeMaxTime','escapeRadius','escapeWaypointTol', ...
        'escapeTargetBias','wObsMax','kHold','kHoldD','nObstacles','obsRMin', ...
        'obsRMax','corridorBand','marginStart','marginGoal','marginObs', ...
        'pathGridResolution','pathClearance','maxObstacleSetAttempts', ...
        'maxObstaclePlacementAttempts','animate','adp'};
    for k = 1:numel(required)
        if ~isfield(params, required{k})
            error('UAV:MissingParameter', '缺少必要参数 params.%s。', required{k});
        end
    end

    mustBePositiveScalar(params.dt, 'dt');
    mustBePositiveScalar(params.Tmax, 'Tmax');
    mustBePositiveScalar(params.mass, 'mass');
    mustBePositiveScalar(params.vmax, 'vmax');
    mustBePositiveScalar(params.amax, 'amax');
    mustBePositiveScalar(params.rho0, 'rho0');
    mustBeNonnegativeScalar(params.obsSafe, 'obsSafe');
    mustBePositiveScalar(params.uavSafe, 'uavSafe');
    mustBePositiveScalar(params.uavAvoidSafe, 'uavAvoidSafe');
    mustBePositiveScalar(params.goalTol, 'goalTol');
    mustBePositiveScalar(params.holdTriggerDist, 'holdTriggerDist');
    mustBePositiveScalar(params.holdDuration, 'holdDuration');
    mustBePositiveScalar(params.formSpacing, 'formSpacing');
    mustBePositiveScalar(params.formWing, 'formWing');
    mustBePositiveScalar(params.pathGridResolution, 'pathGridResolution');
    mustBeNonnegativeScalar(params.pathClearance, 'pathClearance');

    nonnegativeFields = {'kForm','kFormD','formGainBoost','formDampBoost', ...
        'navGain','navAccel','closeBoostGain','kRep','desiredClear','kUAV', ...
        'kUAVVel','collisionAvoidGain','collisionVelGain','collisionMargin', ...
        'kTang','goalDecay','escapeThresh','escapeTargetBias','kHold','kHoldD', ...
        'corridorBand','marginStart','marginGoal','marginObs'};
    for k = 1:numel(nonnegativeFields)
        mustBeNonnegativeScalar(params.(nonnegativeFields{k}), nonnegativeFields{k});
    end
    positiveFields = {'formErrScale','formTol','vTol','closeBoostDist', ...
        'goalScale','escapeWindow','escapeFlipTime','escapeMaxTime', ...
        'escapeRadius','escapeWaypointTol'};
    for k = 1:numel(positiveFields)
        mustBePositiveScalar(params.(positiveFields{k}), positiveFields{k});
    end
    if params.wObsMax < 0 || params.wObsMax > 1 || ...
            params.minGoalRepFactor < 0 || params.minGoalRepFactor > 1
        error('UAV:InvalidParameter', 'wObsMax 和 minGoalRepFactor 必须位于 [0,1]。');
    end
    if ~(isscalar(params.useTangential) && ...
            (islogical(params.useTangential) || any(params.useTangential == [0 1])))
        error('UAV:InvalidParameter', 'useTangential 必须为逻辑值。');
    end
    if ~(isscalar(params.animate) && ...
            (islogical(params.animate) || any(params.animate == [0 1])))
        error('UAV:InvalidParameter', 'animate 必须为逻辑值。');
    end
    if ~(ischar(params.softwareName) || (isstring(params.softwareName) && isscalar(params.softwareName))) || ...
            ~(ischar(params.version) || (isstring(params.version) && isscalar(params.version)))
        error('UAV:InvalidParameter', 'softwareName 和 version 必须是文本标量。');
    end

    if params.holdTriggerDist < params.goalTol
        error('UAV:InvalidParameter', 'holdTriggerDist 必须不小于 goalTol。');
    end
    if params.uavAvoidSafe <= params.uavSafe
        error('UAV:InvalidParameter', 'uavAvoidSafe 必须大于 uavSafe。');
    end
    if ~isscalar(params.nUAV) || params.nUAV < 1 || params.nUAV ~= fix(params.nUAV)
        error('UAV:InvalidParameter', 'nUAV 必须是正整数。');
    end
    if ~isscalar(params.leaderIdx) || params.leaderIdx ~= 1
        error('UAV:InvalidParameter', '当前领航-跟随实现要求 leaderIdx 为 1。');
    end
    if ~isequal(size(params.workspace), [2 2]) || any(~isfinite(params.workspace), 'all')
        error('UAV:InvalidParameter', 'workspace 必须是有限的 2x2 矩阵。');
    end
    lo = params.workspace(1, :).';
    hi = params.workspace(2, :).';
    if any(hi <= lo)
        error('UAV:InvalidParameter', 'workspace 右上角必须逐坐标大于左下角。');
    end
    if ~isequal(size(params.start), [2 1]) || ~isequal(size(params.target), [2 1]) || ...
            any(~isfinite([params.start; params.target]))
        error('UAV:InvalidParameter', 'start 和 target 必须是有限的 2x1 向量。');
    end
    if any(params.start <= lo) || any(params.start >= hi) || ...
            any(params.target <= lo) || any(params.target >= hi)
        error('UAV:InvalidParameter', 'start 和 target 必须严格位于工作空间内部。');
    end
    if norm(params.target - params.start) <= eps
        error('UAV:InvalidParameter', 'start 与 target 不能重合。');
    end

    integerFields = {'nObstacles','maxObstacleSetAttempts','maxObstaclePlacementAttempts'};
    for k = 1:numel(integerFields)
        value = params.(integerFields{k});
        if ~isscalar(value) || value < 0 || value ~= fix(value)
            error('UAV:InvalidParameter', '%s 必须是非负整数。', integerFields{k});
        end
    end
    if params.maxObstacleSetAttempts < 1 || params.maxObstaclePlacementAttempts < 1
        error('UAV:InvalidParameter', '障碍物生成尝试次数必须至少为 1。');
    end
    if params.obsRMin <= 0 || params.obsRMax < params.obsRMin
        error('UAV:InvalidParameter', '障碍物半径范围无效。');
    end

    if ~isfield(params.adp, 'wC') || ~isequal(size(params.adp.wC), [6 1]) || ...
            ~isfield(params.adp, 'wA') || ~isequal(size(params.adp.wA), [2 6])
        error('UAV:InvalidParameter', 'ADP 权重尺寸必须为 wC=6x1、wA=2x6。');
    end
    if any(~isfinite(params.adp.wC), 'all') || any(~isfinite(params.adp.wA), 'all')
        error('UAV:InvalidParameter', 'ADP 初始权重必须全部为有限数。');
    end
    adpRequired = {'enable','gain','uMax','gamma','lrC','lrA','eps','qRisk', ...
        'qForm','qCtrl','qTime','wC','wA','maxWeightNorm'};
    for k = 1:numel(adpRequired)
        if ~isfield(params.adp, adpRequired{k})
            error('UAV:MissingParameter', '缺少必要参数 params.adp.%s。', adpRequired{k});
        end
    end
    if ~isscalar(params.adp.gamma) || ~isfinite(params.adp.gamma) || ...
            params.adp.gamma < 0 || params.adp.gamma > 1
        error('UAV:InvalidParameter', 'adp.gamma 必须位于 [0,1]。');
    end
    adpNonnegative = {'gain','uMax','lrC','lrA','qRisk','qForm','qCtrl','qTime'};
    for k = 1:numel(adpNonnegative)
        mustBeNonnegativeScalar(params.adp.(adpNonnegative{k}), ...
            ['adp.' adpNonnegative{k}]);
    end
    mustBePositiveScalar(params.adp.eps, 'adp.eps');
    mustBePositiveScalar(params.adp.maxWeightNorm, 'adp.maxWeightNorm');
    if ~(isscalar(params.adp.enable) && ...
            (islogical(params.adp.enable) || any(params.adp.enable == [0 1])))
        error('UAV:InvalidParameter', 'adp.enable 必须为逻辑值。');
    end

    if ~isfield(params, 'resultsRoot') || isempty(params.resultsRoot)
        if isfield(params, 'figDir')
            params.resultsRoot = params.figDir;
        else
            error('UAV:MissingParameter', '缺少结果输出目录。');
        end
    end
    if ~isfield(params, 'figDir') || isempty(params.figDir)
        params.figDir = params.resultsRoot;
    end
    if ~(ischar(params.resultsRoot) || ...
            (isstring(params.resultsRoot) && isscalar(params.resultsRoot))) || ...
            ~(ischar(params.figDir) || (isstring(params.figDir) && isscalar(params.figDir)))
        error('UAV:InvalidParameter', 'resultsRoot 和 figDir 必须是文本路径。');
    end
    if isfield(params, 'seed') && ...
            (~isscalar(params.seed) || ~isfinite(params.seed) || params.seed ~= fix(params.seed))
        error('UAV:InvalidParameter', 'seed 必须是有限整数。');
    end
    if ~isfield(params, 'debug')
        params.debug = false;
    end
end

function mustBePositiveScalar(value, name)
    if ~isscalar(value) || ~isfinite(value) || value <= 0
        error('UAV:InvalidParameter', '%s 必须是有限正数。', name);
    end
end

function mustBeNonnegativeScalar(value, name)
    if ~isscalar(value) || ~isfinite(value) || value < 0
        error('UAV:InvalidParameter', '%s 必须是有限非负数。', name);
    end
end

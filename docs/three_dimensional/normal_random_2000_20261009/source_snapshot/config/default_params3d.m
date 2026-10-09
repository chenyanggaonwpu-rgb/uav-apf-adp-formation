function p = default_params3d()
% DEFAULT_PARAMS3D Independent 3-D configuration. Historical 2-D defaults remain frozen.
    p = default_params();
    p.version = '2.3.3';
    p.workspace = [0 0 0; 80 80 30];
    p.start = [10; 10; 10];
    p.target = [70; 62; 10];
    p.heightLayers = [5 10 15 20 25];
    p.adaptiveHeight=struct('enable',true,'maxAddedLayers',8,'intervalTolerance',1e-7);
    p.pathGridResolution = 1;
    p.pathClearance = 1.2; % Highest observed success in the frozen 49-combination scan.
    p.desiredClear = 4.0;
    p.directionalClearance=struct('enable',false, ...
        'planningZ',p.pathClearance,'referenceZ',p.desiredClear);
    p.vmax = 3;
    p.amax = 3;
    p.dt = 0.05;
    p.Tmax = 120;
    p.animate = false;
    p.sceneId = 'S8_window';
    p.nObstacles = 0; % Historical 2-D generator is not used by 3-D scenes.
    p.randomObstacleCount = 8;
    p.normalScene = default_normal_scene_options3d();
    p.verticalCostWeight = 1; % J = 3-D path length + weight * total altitude change.
    p.waypointTol = 1.5;
    p.replanAfter = 4;
    p.terminalBlendTime = 2;
    p.noiseStd = 0;
    p.disturbanceMax = 0;
    p.disturbanceHoldSteps = 1;
    p.safety.enable = true;
    p.safety.maxIterations = 5;
    p.safety.margin = 0.5;
    p.adp.wC = zeros(6, 1);
    p.adp.wA = zeros(3, 6);
end

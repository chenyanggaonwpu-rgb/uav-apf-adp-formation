function [scene,metadata]=random_solids3d(seed,p,options)
% RANDOM_SOLIDS3D Historical unconditioned distribution. No endpoint or flight filtering.
    if nargin<3 || isempty(options)
        defaults=default_normal_scene_options3d(); options=defaults.sampling;
    end
    stream=RandStream('mt19937ar','Seed',seed);
    n=randi(stream,options.countRange);
    empty=struct('type',{},'center',{},'r',{},'zmin',{},'zmax',{},'halfSize',{});
    scene=struct('id','unstructured_random','kind','random','seed',seed, ...
        'obstacles',empty,'family','unstructured', ...
        'description','IID equal-probability types; uniform count and bounded sizes/positions; no endpoint, overlap or route rejection.');
    lo=p.workspace(1,:)'; hi=p.workspace(2,:)'; counts=zeros(1,3); nominalVolume=0;
    for k=1:n
        kind=randi(stream,3); counts(kind)=counts(kind)+1;
        if kind==1
            radius=draw(options.sphereRadius); half=radius*ones(3,1);
            centre=position(half); o=obstacle3d('sphere',centre,radius);
            volume=4*pi*radius^3/3;
        elseif kind==2
            radius=draw(options.cylinderRadius); height=draw(options.cylinderHeight);
            half=[radius;radius;height/2]; centre=position(half);
            o=obstacle3d('cylinder',centre(1:2),radius,centre(3)-height/2,centre(3)+height/2);
            volume=pi*radius^2*height;
        else
            sides=[draw(options.boxWidth);draw(options.boxDepth);draw(options.boxHeight)];
            half=sides/2; centre=position(half); o=obstacle3d('box',centre-half,centre+half);
            volume=prod(sides);
        end
        scene.obstacles(end+1)=o; %#ok<AGROW>
        nominalVolume=nominalVolume+volume;
    end
    metadata=struct('obstacleCount',n,'sphereCount',counts(1),'cylinderCount',counts(2), ...
        'boxCount',counts(3),'nominalVolume',nominalVolume, ...
        'nominalVolumeFraction',nominalVolume/prod(hi-lo),'placementAttempts',n,'rejectedObstacles',0);
    function value=draw(range)
        value=range(1)+(range(2)-range(1))*rand(stream);
    end
    function centre=position(half)
        assert(all(hi-lo>=2*half),'UAV3D:SizeRange','Sampled solid cannot fit in the workspace.');
        centre=lo+half+(hi-lo-2*half).*rand(stream,3,1);
    end
end

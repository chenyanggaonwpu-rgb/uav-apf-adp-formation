function [scene,metadata]=unstructured_scene3d(seed,p,options)
% UNSTRUCTURED_SCENE3D Normal random maps by default; explicit historical sampling stays raw.
% Old protocols supply the complete flat size/count options. Preserve their
% unconditioned geometry. New normal options use mode='normal' and nested sampling.
    if nargin<3 || isempty(options)
        [scene,metadata]=normal_scene3d(seed,p); return;
    end
    if isfield(options,'mode')
        mode=options.mode; options=rmfield(options,'mode');
    else
        mode='unconditioned'; % Compatibility with recorded 2.3.1 protocol.generation.
    end
    if strcmp(mode,'normal')
        [scene,metadata]=normal_scene3d(seed,p,options);
    elseif strcmp(mode,'unconditioned')
        defaults=default_normal_scene_options3d(); sampling=defaults.sampling;
        names=fieldnames(options);
        for k=1:numel(names)
            assert(isfield(sampling,names{k}),'UAV3D:NormalOptions','Unknown sampling option.');
            sampling.(names{k})=options.(names{k});
        end
        [scene,metadata]=random_solids3d(seed,p,sampling);
    else
        error('UAV3D:NormalOptions','Unknown random generation mode: %s',mode);
    end
end

function residual = adp_residual_limit3d(residual,base,p)
% Bounded residual authority around the independently functioning controller.
% A quarter of the bounded base preserves its direction before safety repair.
    maximum=p.adp.gain*p.adp.uMax;
    if isfield(p.adp,'mode') && ismember(p.adp.mode,{'tracking_value_v2','tracking_value_v3','tracking_value_v4'})
        maximum=min(maximum,p.adp.residualBaseFraction*norm(limit_vector3d(base,p.amax)));
    end
    residual=limit_vector3d(residual,maximum);
end

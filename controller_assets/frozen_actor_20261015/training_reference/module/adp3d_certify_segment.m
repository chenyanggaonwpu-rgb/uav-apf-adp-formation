function [safe,margin] = adp3d_certify_segment(a,b,obstacles,p,settings)
% 静态几何证书：有符号距离是 1-Lipschitz；中点距离减半段长为下界。
% 仅在下界严格超过 obsSafe 时放行；达到细分上限仍不确定则拒绝。
% 此处的 margin 是保守下界，不把数值优化的最小值当作安全证明。
    lo=p.workspace(1,:)'+settings.boundaryBuffer;
    hi=p.workspace(2,:)'-settings.boundaryBuffer;
    margin=min([a-lo;b-lo;hi-a;hi-b]);
    safe=margin>settings.tolerance;
    if ~safe, return; end
    for k=1:numel(obstacles)
        [ok,lower]=certifyOne(a,b,obstacles(k),p.obsSafe,settings,0);
        margin=min(margin,lower);
        if ~ok, safe=false; return; end
    end
end

function [safe,lower]=certifyOne(a,b,obstacle,clearance,s,depth)
    midpoint=(a+b)/2;
    distance=obstacle_distance3d(midpoint,obstacle);
    lower=distance-norm(b-a)/2-clearance;
    if lower>s.tolerance, safe=true; return; end
    if distance<=clearance+s.tolerance || depth>=s.maxSubdivisionDepth
        safe=false; return;
    end
    [ok1,m1]=certifyOne(a,midpoint,obstacle,clearance,s,depth+1);
    if ~ok1, safe=false; lower=m1; return; end
    [ok2,m2]=certifyOne(midpoint,b,obstacle,clearance,s,depth+1);
    safe=ok2;
    lower=min(m1,m2);
end

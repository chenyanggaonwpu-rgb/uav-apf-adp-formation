function [safe,margin]=f5_static_cert(a,b,obstacles,p,s)
% Batch the same 1-Lipschitz midpoint lower bound; delegate uncertain segments
% to the unchanged recursive certificate. This changes no acceptance rule.
    lo=p.workspace(1,:)'+s.boundaryBuffer;hi=p.workspace(2,:)'-s.boundaryBuffer;
    boundary=min([a-lo;b-lo;hi-a;hi-b],[],1);
    if any(boundary<=s.tolerance),safe=false;margin=min(boundary);return;end
    lower=point_clearance_batch3d((a+b)/2,obstacles)-vecnorm(b-a)/2-p.obsSafe;
    for i=find(lower<=s.tolerance)
        [ok,m]=adp3d_certify_segment(a(:,i),b(:,i),obstacles,p,s);
        lower(i)=m;if ~ok,safe=false;margin=min([boundary,lower]);return;end
    end
    safe=true;margin=min([boundary,lower]);
end

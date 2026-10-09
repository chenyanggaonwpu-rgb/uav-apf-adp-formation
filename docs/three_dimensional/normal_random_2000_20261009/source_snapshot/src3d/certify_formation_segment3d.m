function [safe,lowerBound]=certify_formation_segment3d(a,b,offsets,obstacles,p,clearance,boundary,tolerance,da,db)
% CERTIFY_FORMATION_SEGMENT3D Conservative 1-Lipschitz certificate for five swept paths.
% All UAVs translate together; pair distances are invariant. Endpoint bounds certify
% the convex workspace. Unresolved intervals are rejected, never declared safe.
    if nargin<9, tolerance=1e-5; end
    pointsA=a+offsets; pointsB=b+offsets;
    lo=p.workspace(1,:)'+boundary; hi=p.workspace(2,:)'-boundary;
    safe=false; lowerBound=-inf;
    if any(pointsA<lo-1e-12,'all') || any(pointsA>hi+1e-12,'all') || ...
            any(pointsB<lo-1e-12,'all') || any(pointsB>hi+1e-12,'all')
        return;
    end
    if nargin<10, da=min(point_clearance_batch3d(pointsA,obstacles)); end
    if nargin<11, db=min(point_clearance_batch3d(pointsB,obstacles)); end
    [safe,lowerBound]=interval(a,b,da,db,0);
    function [ok,lb]=interval(x,y,dx,dy,depth)
        length=norm(y-x); lb=(dx+dy-length)/2;
        if min(dx,dy)<clearance
            ok=false; return;
        end
        if lb>=clearance
            ok=true; return;
        end
        if length<=tolerance || depth>=30
            ok=false; return;
        end
        mid=(x+y)/2; dm=min(point_clearance_batch3d(mid+offsets,obstacles));
        if dm<clearance, ok=false; lb=dm; return; end
        [okLeft,lbLeft]=interval(x,mid,dx,dm,depth+1);
        if ~okLeft, ok=false; lb=lbLeft; return; end
        [okRight,lbRight]=interval(mid,y,dm,dy,depth+1);
        ok=okRight; lb=min(lbLeft,lbRight);
    end
end

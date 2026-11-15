function [hit, minObstacle, minPair, boundaryHit] = check_swept3d(oldState,newState,obstacles,p)
% CHECK_SWEPT3D Evaluate synchronized continuous segments for every UAV.
    n=size(oldState,2);
    minObstacle=inf;
    minPair=inf;
    boundaryHit=false;
    lo=p.workspace(1,:)'; hi=p.workspace(2,:)';
    for i=1:n
        a=oldState(1:3,i); b=newState(1:3,i);
        minObstacle=min(minObstacle,segment_clearance3d(a,b,obstacles));
        if any(a<lo) || any(a>hi) || any(b<lo) || any(b>hi)
            boundaryHit=true;
        end
        for j=i+1:n
            r0=a-oldState(1:3,j);
            dr=(b-newState(1:3,j))-r0;
            t=max(0,min(1,-dot(r0,dr)/max(dot(dr,dr),eps)));
            minPair=min(minPair,norm(r0+t*dr));
        end
    end
    hit=boundaryHit || minObstacle<=p.obsSafe || minPair<=p.uavSafe;
end

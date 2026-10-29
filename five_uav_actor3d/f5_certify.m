function [safe,margin,pairDistance]=f5_certify(a,b,obstacles,p,s)
% Every body segment uses the frozen conservative static-geometry certificate.
% Synchronized pair separation is minimized analytically over the full segment.
    [safe,margin]=f5_static_cert(a(1:3,:),b(1:3,:),obstacles,p,s);
    pairDistance=inf;if ~safe,return;end
    for i=1:p.nUAV
        for j=i+1:p.nUAV
            r=a(1:3,i)-a(1:3,j);dr=(b(1:3,i)-b(1:3,j))-r;
            alpha=max(0,min(1,-dot(r,dr)/max(dot(dr,dr),eps)));
            d=norm(r+alpha*dr);pairDistance=min(pairDistance,d);
            margin=min(margin,d-p.uavSafe);
            if d<=p.uavSafe+s.tolerance,safe=false;return;end
        end
    end
end

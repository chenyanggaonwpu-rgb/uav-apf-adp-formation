function clearance = segment_clearance3d(a,b,obstacles)
% SEGMENT_CLEARANCE3D Minimum swept point-to-solid signed distance.
% Sphere is analytic. Cylinder/box use bounded 1-D minimization after a
% 1-Lipschitz certificate rejects segments that are clearly farther away.
    clearance = inf;
    delta = b-a;
    length2 = dot(delta,delta);
    for k=1:numel(obstacles)
        o=obstacles(k);
        if strcmp(o.type,'sphere')
            t=max(0,min(1,dot(o.center(:)-a,delta)/max(length2,eps)));
            d=norm(a+t*delta-o.center(:))-o.r;
        else
            d0=obstacle_distance3d(a,o);
            d1=obstacle_distance3d(b,o);
            dm=obstacle_distance3d((a+b)/2,o);
            lower=max([d0-norm(delta),d1-norm(delta),dm-norm(delta)/2]);
            if lower>=clearance
                continue;
            end
            if norm(delta)<1e-12
                d=d0;
            else
                opts=optimset('TolX',1e-6,'Display','off');
                [~,dmin]=fminbnd(@(t) obstacle_distance3d(a+t*delta,o),0,1,opts);
                d=min([d0,d1,dm,dmin]);
            end
        end
        clearance=min(clearance,d);
    end
end

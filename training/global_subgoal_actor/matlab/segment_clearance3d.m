function clearance=segment_clearance3d(a,b,obstacles)
% Same distance/minimizer as the frozen implementation. A vectorized signed
% AABB lower bound skips only solids the original Lipschitz test also rejects.
    clearance=inf;delta=b-a;length2=dot(delta,delta);length=norm(delta);
    bounds=dv2_obstacle_bounds(obstacles);
    q=abs((a+b)/2-bounds.center)-bounds.half;
    lowerBounds=vecnorm(max(q,0))+min(max(q,[],1),0)-length/2;
    for k=1:numel(obstacles)
        if lowerBounds(k)>clearance+1e-10,continue;end
        o=obstacles(k);
        if strcmp(o.type,'sphere')
            t=max(0,min(1,dot(o.center(:)-a,delta)/max(length2,eps)));
            d=norm(a+t*delta-o.center(:))-o.r;
        else
            d0=obstacle_distance3d(a,o);d1=obstacle_distance3d(b,o);dm=obstacle_distance3d((a+b)/2,o);
            lower=max([d0-length,d1-length,dm-length/2]);
            if lower>=clearance,continue;end
            if length<1e-12,d=d0;else
                opts=optimset('TolX',1e-6,'Display','off');
                [~,dmin]=fminbnd(@(t) obstacle_distance3d(a+t*delta,o),0,1,opts);
                d=min([d0,d1,dm,dmin]);
            end
        end
        clearance=min(clearance,d);
    end
end

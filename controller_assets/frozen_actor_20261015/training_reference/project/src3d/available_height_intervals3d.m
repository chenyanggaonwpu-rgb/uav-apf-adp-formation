function intervals=available_height_intervals3d(xy,obstacles,p,clearance)
% AVAILABLE_HEIGHT_INTERVALS3D Exact vertical free intervals at a given XY.
% Use the planner's isotropic or XY/Z ellipsoidal margins, not scene labels.
% This is a local section; graph edges must still certify a complete route.
    if nargin<4, clearance=p.pathClearance; end
    xy=xy(:); h=clearance; v=clearance;
    directional=isfield(p,'directionalClearance') && p.directionalClearance.enable;
    if directional
        v=p.directionalClearance.planningZ;
        if clearance~=p.pathClearance, v=min(v,clearance); end
    end
    tol=1e-7;
    if isfield(p,'adaptiveHeight'), tol=p.adaptiveHeight.intervalTolerance; end
    domain=[p.workspace(1,3)+v,p.workspace(2,3)-v];
    intervals=zeros(0,2); blocked=zeros(0,2);
    if domain(2)-domain(1)<=tol, return; end
    for o=obstacles
        switch o.type
            case 'box'
                q=norm(max(abs(xy-o.center(1:2))-o.halfSize(1:2),0))/h;
                if q>=1, continue; end
                pad=v*sqrt(max(0,1-q*q)); band=[o.zmin-pad,o.zmax+pad];
            case 'cylinder'
                q=max(norm(xy-o.center(:))-o.r,0)/h;
                if q>=1, continue; end
                pad=v*sqrt(max(0,1-q*q)); band=[o.zmin-pad,o.zmax+pad];
            case 'sphere'
                radial=norm(xy-o.center(1:2));
                if h==v
                    if radial>=o.r+h, continue; end
                    pad=sqrt(max(0,(o.r+h)^2-radial^2));
                else
                    margins=[h;h;v];
                    if directional_clearance3d([xy;o.center(3)],o,margins)>=1, continue; end
                    lower=0; upper=o.r+v;
                    for iteration=1:36
                        middle=(lower+upper)/2;
                        if directional_clearance3d([xy;o.center(3)+middle],o,margins)<1
                            lower=middle;
                        else
                            upper=middle;
                        end
                    end
                    pad=upper; % Conservative outward root bracket.
                end
                band=o.center(3)+[-pad,pad];
            otherwise
                error('UAV3D:ObstacleType','Unsupported height-section obstacle.');
        end
        band=[max(domain(1),band(1)),min(domain(2),band(2))];
        if band(2)>band(1), blocked(end+1,:)=band; end %#ok<AGROW>
    end
    if isempty(blocked), intervals=domain; return; end
    blocked=sortrows(blocked,[1 2]); cursor=domain(1);
    for k=1:size(blocked,1)
        if blocked(k,1)>cursor+tol
            intervals(end+1,:)=[cursor,blocked(k,1)]; %#ok<AGROW>
        end
        cursor=max(cursor,blocked(k,2));
    end
    if domain(2)>cursor+tol, intervals(end+1,:)=[cursor,domain(2)]; end
end

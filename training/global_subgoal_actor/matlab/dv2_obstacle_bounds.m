function bounds=dv2_obstacle_bounds(obstacles)
% Tight AABBs are lower bounds on distance to the closed obstacle solids.
    bounds.center=zeros(3,numel(obstacles));bounds.half=zeros(3,numel(obstacles));
    types={obstacles.type};
    ids=find(strcmp(types,'box'));
    if ~isempty(ids),bounds.center(:,ids)=reshape([obstacles(ids).center],3,[]);bounds.half(:,ids)=reshape([obstacles(ids).halfSize],3,[]);end
    ids=find(strcmp(types,'sphere'));
    if ~isempty(ids),bounds.center(:,ids)=reshape([obstacles(ids).center],3,[]);bounds.half(:,ids)=repmat([obstacles(ids).r],3,1);end
    ids=find(strcmp(types,'cylinder'));
    if ~isempty(ids)
        bounds.center(:,ids)=[reshape([obstacles(ids).center],2,[]);([obstacles(ids).zmin]+[obstacles(ids).zmax])/2];
        bounds.half(:,ids)=[[obstacles(ids).r];[obstacles(ids).r];([obstacles(ids).zmax]-[obstacles(ids).zmin])/2];
    end
end

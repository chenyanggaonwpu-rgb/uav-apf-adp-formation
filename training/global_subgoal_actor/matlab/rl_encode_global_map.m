function features=rl_encode_global_map(obstacles,p)
% Exact static geometry available to the global Actor, including occluded
% and distant obstacles. No planner path, potential field or teacher label.
    capacity=128;scale=[120;120;30];n=numel(obstacles);
    assert(n<=capacity,'RL:GlobalMapOverflow','Too many obstacles; truncation is forbidden.');
    rows=zeros(n,10);
    for k=1:n
        o=obstacles(k);types=double(strcmp(o.type,{'box','cylinder','sphere'}));
        assert(sum(types)==1,'RL:GlobalPrimitive','Unsupported primitive.');
        switch o.type
            case 'box',center=double(o.center(:));extent=double(o.halfSize(:));
            case 'cylinder'
                center=[double(o.center(:));(o.zmin+o.zmax)/2];
                extent=[o.r;o.r;(o.zmax-o.zmin)/2];
            case 'sphere',center=double(o.center(:));extent=repmat(double(o.r),3,1);
        end
        assert(all(isfinite(center))&&all(isfinite(extent))&&all(extent>0));
        rows(k,:)=[1,types,(center./scale)',(extent./scale)'];
    end
    % Sort the serialized precision so sub-ulp box-center reconstruction
    % differences cannot move otherwise tied tokens into different slots.
    rows=double(single(rows));rows=sortrows(rows,[5 6 7 2 3 4 8 9 10]);tokens=zeros(10,capacity,'single');
    tokens(:,1:n)=single(rows');
    bounds=double(p.workspace);features=single([bounds(1,:)'./scale;bounds(2,:)'./scale;double(p.target(:))./scale;double(tokens(:))]);
    assert(isequal(size(features),[1289 1])&&all(isfinite(features)));
end

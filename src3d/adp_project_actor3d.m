function weights = adp_project_actor3d(weights)
% Constrained policy class: restoring-position and velocity-matching blocks.
% Stored 3x6 weights remain compatible; each symmetric PSD block has six DOF.
    for columns={1:3,4:6}
        index=columns{1};block=weights(:,index);
        block=.5*(block+block');
        [basis,values]=eig(block);
        weights(:,index)=basis*max(0,values)*basis';
    end
end

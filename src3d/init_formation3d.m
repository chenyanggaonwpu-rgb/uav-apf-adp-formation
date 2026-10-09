function [state,offsets,psi] = init_formation3d(p)
% INIT_FORMATION3D Horizontal V with all vehicles at the leader's height.
    psi=atan2(p.target(2)-p.start(2),p.target(1)-p.start(1));
    rot=[cos(psi) -sin(psi);sin(psi) cos(psi)];
    offsets=zeros(2,p.nUAV);
    state=zeros(6,p.nUAV);
    for i=1:p.nUAV
        if i>1
            row=ceil((i-1)/2);
            side=2*mod(i,2)-1;
            offsets(:,i)=[-row*p.formSpacing;side*row*p.formWing];
        end
        state(1:3,i)=[p.start(1:2)+rot*offsets(:,i);p.start(3)];
    end
    lo=p.workspace(1,:)';hi=p.workspace(2,:)';
    terminal=[p.target(1:2)+rot*offsets;p.target(3)*ones(1,p.nUAV)];
    if any(state(1:3,:)<lo,'all') || any(state(1:3,:)>hi,'all') || ...
            any(terminal<lo,'all') || any(terminal>hi,'all')
        error('UAV3D:FormationOutOfBounds','Initial or final V lies outside the workspace.');
    end
end

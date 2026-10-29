function draw_obstacles3d(ax,obstacles,threeDimensional,alpha)
% DRAW_OBSTACLES3D Shared closed-solid rendering for plots and animations.
% A top view is the projection of all heights, not a 2-D collision map.
    if nargin<4, alpha=0.25; end
    for k=1:numel(obstacles)
        o=obstacles(k);
        color=[0.55 0.59 0.65];
        if ~strcmp(o.type,'sphere') && o.zmin>0, color=[0.24 0.43 0.66]; end
        if threeDimensional
            switch o.type
                case 'sphere'
                    [x,y,z]=sphere(20);
                    surf(ax,o.center(1)+o.r*x,o.center(2)+o.r*y, ...
                        o.center(3)+o.r*z,'FaceColor',color, ...
                        'FaceAlpha',alpha,'EdgeColor','none');
                case 'cylinder'
                    [x,y,z]=cylinder(o.r,32);
                    surf(ax,o.center(1)+x,o.center(2)+y, ...
                        o.zmin+(o.zmax-o.zmin)*z,'FaceColor',color, ...
                        'FaceAlpha',alpha,'EdgeColor','none');
                    theta=linspace(0,2*pi,33);
                    for zcap=[o.zmin o.zmax]
                        fill3(ax,o.center(1)+o.r*cos(theta), ...
                            o.center(2)+o.r*sin(theta),zcap*ones(size(theta)), ...
                            color,'FaceAlpha',alpha,'EdgeColor','none');
                    end
                case 'box'
                    lo=o.center-o.halfSize; hi=o.center+o.halfSize;
                    vertices=[lo(1) lo(2) lo(3);hi(1) lo(2) lo(3); ...
                        hi(1) hi(2) lo(3);lo(1) hi(2) lo(3); ...
                        lo(1) lo(2) hi(3);hi(1) lo(2) hi(3); ...
                        hi(1) hi(2) hi(3);lo(1) hi(2) hi(3)];
                    faces=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
                    patch(ax,'Vertices',vertices,'Faces',faces,'FaceColor',color, ...
                        'FaceAlpha',alpha,'EdgeColor',color,'EdgeAlpha',0.35);
                otherwise
                    error('UAV3D:ObstacleType','Unsupported obstacle type: %s',o.type);
            end
        else
            if strcmp(o.type,'box')
                lo=o.center-o.halfSize; hi=o.center+o.halfSize;
                fill(ax,[lo(1) hi(1) hi(1) lo(1)], ...
                    [lo(2) lo(2) hi(2) hi(2)],color, ...
                    'FaceAlpha',alpha,'EdgeColor','none');
            elseif strcmp(o.type,'sphere') || strcmp(o.type,'cylinder')
                angle=linspace(0,2*pi,80);
                fill(ax,o.center(1)+o.r*cos(angle),o.center(2)+o.r*sin(angle), ...
                    color,'FaceAlpha',alpha,'EdgeColor','none');
            else
                error('UAV3D:ObstacleType','Unsupported obstacle type: %s',o.type);
            end
        end
    end
end

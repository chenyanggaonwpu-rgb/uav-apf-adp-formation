function apf_adp_plot(automatic,baseline,outputStem)
% Native MATLAB plots, with actual executed paths and logged switching events.
    item=automatic.item; p=item.parameters; states=automatic.states;
    fig=figure('Visible','off','Color','w','Position',[50 50 1400 800]);
    tiledlayout(3,2,'Padding','compact','TileSpacing','compact');
    nexttile(1,[3 1]); hold on;
    for k=1:numel(item.obstacles)
        o=item.obstacles(k);
        switch o.type
            case 'box'
                lo=o.center-o.halfSize; hi=o.center+o.halfSize;
                vertices=[lo';hi(1) lo(2) lo(3);hi(1) hi(2) lo(3);lo(1) hi(2) lo(3); ...
                    lo(1) lo(2) hi(3);hi(1) lo(2) hi(3);hi';lo(1) hi(2) hi(3)];
                patch('Vertices',vertices,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8], ...
                    'FaceColor',[.55 .6 .65],'FaceAlpha',.12,'EdgeAlpha',.2,'HandleVisibility','off');
            case 'sphere'
                [x,y,z]=sphere(20);
                surf(o.center(1)+o.r*x,o.center(2)+o.r*y,o.center(3)+o.r*z, ...
                    'FaceColor',[.55 .6 .65],'FaceAlpha',.18,'EdgeColor','none','HandleVisibility','off');
            case 'cylinder'
                [x,y,z]=cylinder(o.r,24);
                surf(o.center(1)+x,o.center(2)+y,o.zmin+(o.zmax-o.zmin)*z, ...
                    'FaceColor',[.55 .6 .65],'FaceAlpha',.18,'EdgeColor','none','HandleVisibility','off');
        end
    end
    a=plot3(baseline.states(1,:),baseline.states(2,:),baseline.states(3,:), ...
        '--','Color',[.4 .4 .4],'LineWidth',1.4);
    b=plot3(states(1,:),states(2,:),states(3,:),'-','Color',[.04 .35 .65],'LineWidth',1.8);
    active=false(1,size(states,2));
    for k=1:numel(automatic.modes)
        if automatic.modes(k)==2
            indices=(k-1)*4+1:min(k*4+1,size(states,2)); active(indices)=true;
        end
    end
    colored=states; colored(:,~active)=NaN;
    c=plot3(colored(1,:),colored(2,:),colored(3,:),'-','Color',[.88 .4 .05],'LineWidth',2.8);
    d=scatter3(p.target(1),p.target(2),p.target(3),100,[.05 .55 .2],'p','filled');
    for k=1:numel(automatic.events)
        event=automatic.events(k);
        if event.to==2
            scatter3(event.position(1),event.position(2),event.position(3),65,[.75 .15 .12],'o','filled','HandleVisibility','off');
        elseif event.from==2
            scatter3(event.position(1),event.position(2),event.position(3),65,[.1 .6 .3],'s','filled','HandleVisibility','off');
        end
    end
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)'); grid on; axis equal;
    xlim(p.workspace(:,1)'); ylim(p.workspace(:,2)'); zlim(p.workspace(:,3)'); view(42,24);
    title(strrep(item.id,'_',' ')); legend([a b c d],{'Pure APF','Automatic','ADP waypoint active','Goal'},'Location','best');
    horizon=automatic.summary.flightSeconds+3;
    for panel=1:2
        nexttile(panel*2); hold on;
        for version=1:2
            if version==1, episode=baseline; color=[.4 .4 .4]; else, episode=automatic; color=[.04 .35 .65]; end
            if panel==1, value=vecnorm(episode.states(1:3,:)-p.target,2,1);
            else, value=vecnorm(episode.states(4:6,:),2,1); end
            plot((0:numel(value)-1)*p.dt,value,'Color',color,'LineWidth',1.4);
        end
        if panel==1, yline(p.goalTol,':'); ylabel('Goal distance (m)');
        else, yline(p.vTol,':'); ylabel('Speed (m/s)'); end
        xlim([0 horizon]); grid on;
    end
    nexttile(6); hold on;
    times=(0:numel(automatic.modes)-1)*4*p.dt;
    stairs([times automatic.summary.flightSeconds], [automatic.modes automatic.modes(end)], ...
        'Color',[.04 .35 .65],'LineWidth',1.8);
    ylim([.5 5.5]); yticks(1:5); yticklabels({'APF','ADP waypoint','APF cooldown','Terminal APF','Brake failure'});
    xlim([0 horizon]); xlabel('Simulation time (s)'); grid on;
    sgtitle(sprintf('APF goal=%d | Automatic goal=%d | activations=%d | collisions=%d', ...
        baseline.summary.success,automatic.summary.success,automatic.summary.activations,automatic.summary.collision));
    exportgraphics(fig,[outputStem '.png'],'Resolution',150);
    exportgraphics(fig,[outputStem '.pdf'],'ContentType','image','Resolution',180);
    savefig(fig,[outputStem '.fig']); close(fig);
end

function plot_environment_validation3d(studyDir)
% PLOT_ENVIRONMENT_VALIDATION3D Render all fixed maps, including failed plans.
    root=fileparts(fileparts(mfilename('fullpath')));
    oldPath=path; guard=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(root,'src3d'));
    loaded=load(fullfile(studyDir,'protocol.mat')); p=loaded.protocol.p;
    fig=figure('Visible','off','Color','w','Position',[50 50 1440 850]);
    layout=tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');
    ids={'C_S7_underpass','C_S8_window','C_S9_altitude_choice', ...
        'C_S10_layer_gap','C_S11_blocked','C_random_000001'};
    titles={'Pass below a roof','Pass through a height window','Detour or climb', ...
        'Five layers miss the window','Sealed wall / negative control','Mixed suspended solids'};
    for i=1:numel(ids)
        loaded=load(fullfile(studyDir,'trials',[ids{i} '.mat'])); r=loaded.r;
        ax=nexttile(layout); hold(ax,'on'); grid(ax,'on');
        draw_obstacles3d(ax,r.scene.obstacles,true,0.24);
        if ~isempty(r.route)
            route=[p.start,r.route];
            plot3(ax,route(1,:),route(2,:),route(3,:),'--', ...
                'Color',[0.02 0.43 0.82],'LineWidth',1.6);
        end
        if isfield(r.history,'state')
            xyz=squeeze(r.history.state(1:3,1,:));
            plot3(ax,xyz(1,:),xyz(2,:),xyz(3,:),'Color',[0.02 0.62 0.45], ...
                'LineWidth',1.8);
        end
        scatter3(ax,p.start(1),p.start(2),p.start(3),35,[0.02 0.62 0.45],'filled');
        scatter3(ax,p.target(1),p.target(2),p.target(3),45,[0.88 0.23 0.20],'filled');
        xlabel(ax,'x / m'); ylabel(ax,'y / m'); zlabel(ax,'height / m');
        xlim(ax,[0 80]); ylim(ax,[0 80]); zlim(ax,[0 30]);
        view(ax,35,25); daspect(ax,[1 1 0.8]); ax.FontName='Arial';
        title(ax,{titles{i},strrep(r.reason,'_',' ')},'FontSize',12);
    end
    sgtitle(layout,'3-D ENVIRONMENTS  |  Height is a decision','FontWeight','bold');
    exportgraphics(fig,fullfile(studyDir,'environment_overview.png'),'Resolution',160);
    savefig(fig,fullfile(studyDir,'environment_overview.fig')); close(fig);

    failed=load(fullfile(studyDir,'trials','C_S10_layer_gap.mat'));
    added=load(fullfile(studyDir,'trials','S10_added_13m.mat'));
    scene=failed.scene; q=added.q;
    fig=figure('Visible','off','Color','w','Position',[50 50 1200 650]);
    layout=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
    ax=nexttile(layout); hold(ax,'on'); grid(ax,'on');
    draw_obstacles3d(ax,scene.obstacles,true,0.24);
    route=[q.start,added.r.route];
    plot3(ax,route(1,:),route(2,:),route(3,:),'-', ...
        'Color',[0.02 0.62 0.45],'LineWidth',2.5);
    scatter3(ax,q.start(1),q.start(2),q.start(3),55,'g','filled');
    scatter3(ax,q.target(1),q.target(2),q.target(3),55,'r','filled');
    view(ax,35,25); daspect(ax,[1 1 0.8]);
    xlabel(ax,'x / m'); ylabel(ax,'y / m'); zlabel(ax,'height / m');
    xlim(ax,[0 80]); ylim(ax,[0 80]); zlim(ax,[0 30]);
    title(ax,{'Same geometry, different layers', ...
        sprintf('Five layers: %s  |  Add 13 m: %s', ...
        strrep(failed.r.reason,'_',' '),strrep(added.r.reason,'_',' '))});
    ax=nexttile(layout); hold(ax,'on'); grid(ax,'on');
    for o=scene.obstacles
        lo=o.center-o.halfSize; hi=o.center+o.halfSize;
        fill(ax,[lo(1) hi(1) hi(1) lo(1)], ...
            [lo(3) lo(3) hi(3) hi(3)],[0.55 0.59 0.65], ...
            'FaceAlpha',0.55,'EdgeColor','none');
    end
    low=8+p.pathClearance; high=18-p.pathClearance;
    fill(ax,[36 44 44 36],[low low high high],[0.15 0.75 0.50], ...
        'FaceAlpha',0.45,'EdgeColor','none');
    for layer=p.heightLayers
        yline(ax,layer,'--','Color',[0.5 0.5 0.5]);
        text(ax,49,layer+0.4,sprintf('%g m fixed',layer),'FontSize',10);
    end
    yline(ax,13,'-','Color',[0.02 0.62 0.45],'LineWidth',2);
    text(ax,25,13.7,'13 m added','Color',[0.02 0.52 0.37],'FontWeight','bold');
    plot(ax,[35 45],[low low],':','Color',[0.05 0.40 0.30],'LineWidth',1.2);
    plot(ax,[35 45],[high high],':','Color',[0.05 0.40 0.30],'LineWidth',1.2);
    xlabel(ax,'x / m  (section at y = 40 m)'); ylabel(ax,'height / m');
    xlim(ax,[24 57]); ylim(ax,[0 30]);
    title(ax,{'Opening: 8-18 m',sprintf('With %.1f m clearance: %.1f-%.1f m', ...
        p.pathClearance,low,high)});
    sgtitle(layout,'A continuous corridor can fall between the fixed height layers', ...
        'FontWeight','bold');
    exportgraphics(fig,fullfile(studyDir,'layer_gap_comparison.png'),'Resolution',180);
    savefig(fig,fullfile(studyDir,'layer_gap_comparison.fig')); close(fig);

    for id={'C_S7_underpass','C_S8_window','S9_weight_0','S9_weight_2','S10_added_13m'}
        loaded=load(fullfile(studyDir,'trials',[id{1} '.mat']));
        if ~isempty(loaded.r) && isfield(loaded.r.history,'state')
            caseDir=fullfile(studyDir,'cases',id{1}); mkdir(caseDir);
            plot_result3d(loaded.r,caseDir);
        end
    end
end

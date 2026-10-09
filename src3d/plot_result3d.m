function plot_result3d(result,outputDir)
% PLOT_RESULT3D Save 3-D, top-down, height and safety evidence.
    state=result.history.state;
    t=result.history.t;
    p=result.params;
    colors=lines(p.nUAV);
    fig=figure('Visible','off','Color','w','Position',[100 100 1200 850]);
    layout=tiledlayout(fig,2,2,'Padding','compact');
    ax=nexttile(layout,1);hold(ax,'on');grid(ax,'on');
    draw_obstacles3d(ax,result.scene.obstacles,true);
    for i=1:p.nUAV
        points=squeeze(state(1:3,i,:));
        plot3(ax,points(1,:),points(2,:),points(3,:), ...
            'Color',colors(i,:),'LineWidth',1.6);
    end
    scatter3(ax,p.start(1),p.start(2),p.start(3),70,'g','filled');
    scatter3(ax,p.target(1),p.target(2),p.target(3),70,'r','filled');
    xlabel(ax,'x (m)');ylabel(ax,'y (m)');zlabel(ax,'z (m)');
    title(ax,'Three-dimensional formation trajectory');
    view(ax,35,25);axis(ax,'equal');
    ax=nexttile(layout,2);hold(ax,'on');grid(ax,'on');
    draw_obstacles3d(ax,result.scene.obstacles,false);
    for i=1:p.nUAV
        points=squeeze(state(1:3,i,:));
        plot(ax,points(1,:),points(2,:),'Color',colors(i,:),'LineWidth',1.3);
    end
    xlabel(ax,'x (m)');ylabel(ax,'y (m)');
    title(ax,'Top projection (all heights)');axis(ax,'equal');
    ax=nexttile(layout,3);hold(ax,'on');grid(ax,'on');
    for i=1:p.nUAV
        plot(ax,t,squeeze(state(3,i,:)),'Color',colors(i,:),'LineWidth',1.2);
    end
    xlabel(ax,'time (s)');ylabel(ax,'height (m)');title(ax,'Per-vehicle height');
    ax=nexttile(layout,4);hold(ax,'on');grid(ax,'on');
    plot(ax,t,result.history.formationError,'LineWidth',1.2);
    plot(ax,t,result.history.obstacleClearance,'LineWidth',1.2);
    plot(ax,t,result.history.pairDistance,'LineWidth',1.2);
    yline(ax,p.obsSafe,'--');yline(ax,p.uavSafe,'--');
    xlabel(ax,'time (s)');ylabel(ax,'metres');
    legend(ax,{'formation error','obstacle clearance','pair distance', ...
        'obstacle hard distance','pair hard distance'},'Location','best');
    title(ax,'Dynamic reference and safety');
    sgtitle(layout,sprintf('%s | %s',result.scene.id,result.reason), ...
        'Interpreter','none');
    exportgraphics(fig,fullfile(outputDir,'trajectory_3d.png'),'Resolution',160);
    savefig(fig,fullfile(outputDir,'trajectory_3d.fig'));
    close(fig);
    fig=figure('Visible','off','Color','w','Position',[100 100 1100 700]);
    layout=tiledlayout(fig,2,2,'Padding','compact');
    ax=nexttile(layout,1);plot(ax,t,result.history.baseNorm,t,result.history.adpNorm, ...
        t,result.history.safetyNorm);grid(ax,'on');
    legend(ax,{'base','ADP','safety correction'});ylabel(ax,'acceleration (m/s^2)');
    title(ax,'Control contributions');
    ax=nexttile(layout,2);plot(ax,t,result.history.cycleMs);hold(ax,'on');
    yline(ax,50,'--');grid(ax,'on');ylabel(ax,'milliseconds');title(ax,'Full online cycle');
    ax=nexttile(layout,3);stairs(ax,t,double(result.history.safetyTriggered));hold(ax,'on');
    stairs(ax,t,double(result.history.tangentDegenerate));grid(ax,'on');
    legend(ax,{'safety triggered','tangent degenerate'});title(ax,'Event timeline');
    ax=nexttile(layout,4);plot(ax,t,result.history.commandNorm, ...
        t,result.history.actualAccelNorm);grid(ax,'on');
    legend(ax,{'command','actual'});ylabel(ax,'acceleration (m/s^2)');
    title(ax,'Command and actual input');
    exportgraphics(fig,fullfile(outputDir,'diagnostics_3d.png'),'Resolution',160);
    savefig(fig,fullfile(outputDir,'diagnostics_3d.fig'));
    close(fig);
end

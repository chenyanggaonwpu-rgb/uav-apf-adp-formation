function render_3d_animation(source, outputMp4)
% RENDER_3D_ANIMATION Export a 3-D result struct or saved MAT as MP4.
% render_3d_animation(result, 'flight_3d.mp4')
% render_3d_animation('trials/C_S3_overpass.mat', 'S3_overpass.mp4')

    if isstruct(source)
        r = source;
    else
        loaded = load(source);
        if isfield(loaded,'r')
            r = loaded.r;
        elseif isfield(loaded,'result')
            r = loaded.result;
        else
            error('UAV3D:AnimationSource','MAT must contain r or result.');
        end
    end
    [outputParent,~,~] = fileparts(outputMp4);
    if ~isempty(outputParent) && ~isfolder(outputParent), mkdir(outputParent); end
    state = r.history.state;
    t = r.history.t;
    p = r.params;
    n = size(state, 2);
    nFrame = size(state, 3);
    color = [0.08 0.40 0.95; 0.93 0.38 0.22; 0.59 0.36 0.91; ...
             0.03 0.65 0.56; 0.97 0.65 0.12];

    fig = figure('Visible','off','Color','w','Position',[50 50 1280 720], ...
        'Renderer','opengl','InvertHardcopy','off');
    ax = axes('Parent',fig,'Position',[0.055 0.13 0.60 0.75]);
    hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    ax.GridColor = [0.86 0.89 0.92];
    ax.FontName = 'Arial'; ax.FontSize = 10;
    draw_obstacles3d(ax,r.scene.obstacles,true,0.30);
    scatter3(ax,p.start(1),p.start(2),p.start(3),120, ...
        'filled','MarkerFaceColor',[0.16 0.70 0.43]);
    scatter3(ax,p.target(1),p.target(2),p.target(3),140, ...
        'p','filled','MarkerFaceColor',[0.93 0.30 0.27]);
    trails = gobjects(1,n); markers = gobjects(1,n); labels = gobjects(1,n);
    links = gobjects(1,n-1);
    for j = 1:n
        trails(j) = plot3(ax,NaN,NaN,NaN,'Color',color(j,:), 'LineWidth',1.8);
        markers(j) = scatter3(ax,NaN,NaN,NaN,130,'filled', ...
            'MarkerFaceColor',color(j,:),'MarkerEdgeColor','w','LineWidth',1.4);
        labels(j) = text(ax,0,0,0,sprintf('  U%d',j),'FontName','Arial', ...
            'FontSize',10,'FontWeight','bold','Color',color(j,:));
    end
    for j = 2:n
        links(j-1) = plot3(ax,NaN,NaN,NaN,':','Color',[0.47 0.53 0.61], ...
            'LineWidth',1.0);
    end
    xlim(ax,p.workspace(:,1)'); ylim(ax,p.workspace(:,2)'); zlim(ax,p.workspace(:,3)');
    daspect(ax,[1 1 0.8]); view(ax,38,25);
    xlabel(ax,'x / m'); ylabel(ax,'y / m'); zlabel(ax,'z / m');
    title(ax,sprintf('%s  |  Five UAVs in a 3-D environment',r.scene.id), ...
        'FontName','Arial','FontSize',15,'FontWeight','bold','Interpreter','none');

    axH = axes('Parent',fig,'Position',[0.735 0.57 0.22 0.26]);
    hold(axH,'on'); grid(axH,'on'); box(axH,'on');
    for j = 1:n
        plot(axH,t,squeeze(state(3,j,:)),'Color',color(j,:),'LineWidth',1.4);
    end
    cursor = xline(axH,0,'--','Color',[0.10 0.14 0.20],'LineWidth',1.2);
    xlim(axH,[0 max(t)]); ylim(axH,p.workspace(:,3)');
    xlabel(axH,'time / s'); ylabel(axH,'height / m');
    title(axH,'Altitude history','FontName','Arial','FontWeight','bold');
    axH.FontName='Arial'; axH.FontSize=9;

    axT = axes('Parent',fig,'Position',[0.735 0.16 0.22 0.27]);
    hold(axT,'on'); grid(axT,'on'); box(axT,'on');
    draw_obstacles3d(axT,r.scene.obstacles,false,0.3);
    topTrails=gobjects(1,n); topMarkers=gobjects(1,n);
    for j=1:n
        topTrails(j)=plot(axT,NaN,NaN,'Color',color(j,:),'LineWidth',1.3);
        topMarkers(j)=plot(axT,NaN,NaN,'o','MarkerFaceColor',color(j,:), ...
            'MarkerEdgeColor','w','MarkerSize',7);
    end
    xlim(axT,p.workspace(:,1)'); ylim(axT,p.workspace(:,2)'); axis(axT,'square');
    xlabel(axT,'x / m'); ylabel(axT,'y / m');
    title(axT,'Projection / all heights','FontName','Arial','FontWeight','bold');
    axT.FontName='Arial'; axT.FontSize=9;

    timeText = annotation(fig,'textbox',[0.055 0.035 0.30 0.04], ...
        'String','t = 0.0 s','FontName','Arial','FontSize',13, ...
        'FontWeight','bold','EdgeColor','none','Color',[0.12 0.16 0.22]);
    annotation(fig,'textbox',[0.72 0.91 0.26 0.04], ...
        'String','UAV FORMATION  /  3D', 'FontName','Arial', 'FontSize',10, ...
        'FontWeight','bold','EdgeColor','none','Color',[0.33 0.39 0.47]);

    video = VideoWriter(outputMp4,'MPEG-4');
    video.FrameRate=12; video.Quality=84;
    open(video);
    indices=unique([1:4:nFrame,nFrame]);
    for k=indices
        for j=1:n
            xyz=squeeze(state(1:3,j,1:k));
            set(trails(j),'XData',xyz(1,:),'YData',xyz(2,:),'ZData',xyz(3,:));
            set(markers(j),'XData',xyz(1,end),'YData',xyz(2,end), ...
                'ZData',xyz(3,end));
            set(labels(j),'Position',xyz(:,end).'+[0.5 0.5 0.4]);
            set(topTrails(j),'XData',xyz(1,:),'YData',xyz(2,:));
            set(topMarkers(j),'XData',xyz(1,end),'YData',xyz(2,end));
        end
        leader=squeeze(state(1:3,1,k));
        for j=2:n
            follower=squeeze(state(1:3,j,k));
            set(links(j-1),'XData',[leader(1),follower(1)], ...
                'YData',[leader(2),follower(2)], ...
                'ZData',[leader(3),follower(3)]);
        end
        cursor.Value=t(k);
        timeText.String=sprintf('t = %.1f / %.1f s   |   %s',t(k),t(end),r.reason);
        drawnow;
        frame=getframe(fig);
        writeVideo(video,frame);
    end
    close(video);
    close(fig);
end

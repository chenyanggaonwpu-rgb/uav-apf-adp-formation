function plot_margin_grid49_3d(studyDir)
% PLOT_MARGIN_GRID49_3D Numeric coordinates and all 49 observed grid points.
    t=readtable(fullfile(studyDir,'grid_summary.csv'),'VariableNamingRule','preserve');
    p=jsondecode(fileread(fullfile(studyDir,'protocol.json')));
    x=p.pathValues(:)';y=p.referenceValues(:)';[xx,yy]=meshgrid(x,y);
    z=zeros(7,7);zs=z;count=z;timeout=z;safety=z;planned=z;
    for k=1:height(t)
        a=t.pathIndex(k);b=t.referenceIndex(k);
        z(b,a)=t.successPercent(k);zs(b,a)=t.safeSuccessPercent(k);count(b,a)=t.success(k);
        timeout(b,a)=t.timeout(k);safety(b,a)=t.safetyUnresolved(k)+t.collision(k)+ ...
            t.boundaryViolation(k)+t.invalidState(k);planned(b,a)=t.initialPlanFound(k);
    end
    output=fullfile(studyDir,'figures');if ~isfolder(output),mkdir(output);end
    set(groot,'defaultAxesFontName','Microsoft YaHei','defaultTextFontName','Microsoft YaHei');
    set(groot,'defaultAxesFontSize',12,'defaultTextInterpreter','none','defaultAxesTickLabelInterpreter','none');
    palette=[linspace(.91,.12,256)',linspace(.96,.39,256)',linspace(1,.77,256)'];
    surface_figure(z,'01_success_surface_all200','全部 200 张地图 · 完整任务成功率',200,78.5);
    surface_figure(zs,'02_success_surface_safe157','157 张初始安全地图 · 完整任务成功率',157,100);
    f=figure('Visible','off','Color','w','Position',[60 60 1540 770]);
    layout=tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact');
    for k=1:2
        ax=nexttile(layout);v=z;denominator=200;subtitle='全部地图 · 分母 200';
        if k==2,v=zs;denominator=157;subtitle='固定初始安全子集 · 分母 157';end
        heat(ax,v,0,100);title(ax,subtitle,'FontWeight','normal','FontSize',16);
        for a=1:7
            for b=1:7
                color=[.1 .15 .2];if v(b,a)>65,color=[1 1 1];end
                text(ax,a,b,sprintf('%.1f%%\n%d/%d',v(b,a),count(b,a),denominator), ...
                    'HorizontalAlignment','center','FontSize',10,'Color',color);
            end
        end
        rectangle(ax,'Position',[5.5 5.5 1 1],'EdgeColor',[.95 .48 .12],'LineWidth',2);
    end
    title(layout,'49 个实测组合 · 成功率精确查阅','FontSize',21,'FontWeight','normal');
    caption(f,'橙框为原基线 4.2 / 4.0 m。此图各格仅用于查值；三维曲面使用实际数值间距。');
    export(f,'03_success_heatmaps');
    f=figure('Visible','off','Color','w','Position',[60 60 1540 770]);
    layout=tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact');
    for k=1:2
        ax=nexttile(layout);v=timeout;name='飞行超时 / 200';
        if k==2,v=safety;name='安全未解决、碰撞、越界或无效状态 / 200';end
        heat(ax,v,0,max(1,max(v,[],'all')));title(ax,name,'FontWeight','normal','FontSize',15);
        for a=1:7,for b=1:7
            text(ax,a,b,sprintf('%d',v(b,a)),'HorizontalAlignment','center', ...
                'FontSize',12,'Color',pick_color(v(b,a),max(v,[],'all')));
        end,end
    end
    title(layout,'几何通路增加后，完整飞行的失败分布','FontSize',21,'FontWeight','normal');
    caption(f,'左、右面板为计数且各自使用独立色阶；规划失败与 43 张初始不安全地图未计入这里。');
    export(f,'04_flight_failure_heatmaps');
    copyfile(fullfile(studyDir,'grid_summary.csv'),fullfile(output,'figure_source_data.csv'));
    save(fullfile(output,'surface_data.mat'),'x','y','xx','yy','z','zs','count','timeout','safety','planned');
    receipt=struct('figures',4,'measuredPointsPerSurface',49,'numericAxisSpacing',true, ...
        'interpolation','Flat-faced connections between measured grid points; no spline or fitted model.', ...
        'primaryDenominator',200,'fixedInitialSafeDenominator',157);
    fid=fopen(fullfile(output,'plot_receipt.json'),'w','n','UTF-8');
    fprintf(fid,'%s',jsonencode(receipt,PrettyPrint=true));fclose(fid);
    fprintf('MARGIN_GRID_FIGURES_COMPLETE directory=%s\n',output);
    function surface_figure(values,name,heading,denominator,ceiling)
        f=figure('Visible','off','Color','w','Position',[60 60 1500 1020]);
        ax=axes(f,'Position',[.12 .18 .70 .69]);
        surf(ax,xx,yy,values,'FaceColor','flat','EdgeColor',[.36 .43 .51],'LineWidth',.6);hold(ax,'on');
        scatter3(ax,xx(:),yy(:),values(:),28,[.08 .16 .25],'filled');
        base=values(6,6);
        scatter3(ax,4.2,4,base,125,[.95 .48 .12],'filled','MarkerEdgeColor','w');
        text(ax,4.2,4,base+7,sprintf('基线 %.1f%%',base),'Color',[.65 .28 .06], ...
            'FontSize',13,'HorizontalAlignment','left');
        if ceiling<100
            plot3(ax,[x(1) x(end) x(end) x(1) x(1)], ...
                [y(1) y(1) y(end) y(end) y(1)],ceiling*ones(1,5),'--', ...
                'Color',[.45 .49 .54],'LineWidth',1);
        end
        view(ax,[-43 27]);zlim(ax,[0 100]);clim(ax,[0 100]);colormap(ax,palette);
        xlim(ax,[x(1) x(end)]);ylim(ax,[y(1) y(end)]);
        ax.XTick=x;ax.YTick=y;ax.ZTick=0:20:100;
        ax.XTickLabel=compose('%.1f',x);ax.YTickLabel=compose('%.1f',y);
        ax.ZTickLabel=compose('%d%%',0:20:100);ax.GridColor=[.78 .81 .85];ax.GridAlpha=.35;
        ax.Color=[1 1 1];ax.Box='off';ax.LineWidth=.8;
        xlabel(ax,'规划余量 pathClearance / m','FontSize',14);
        ylabel(ax,'参考余量 desiredClear / m','FontSize',14);
        zlabel(ax,'完整任务成功率','FontSize',14);
        title(ax,heading,'FontSize',22,'FontWeight','normal');
        cb=colorbar(ax,'Position',[.87 .25 .019 .5]);cb.Label.String='成功率 / %';cb.Ticks=0:20:100;
        note=sprintf('每个点均执行同一批地图；固定分母 %d。黑点为 49 个实测组合，网格面仅连接采样点。',denominator);
        if ceiling<100,note=[note newline '43 张初始不安全地图全部保留；仅此项就使总体成功率上限不超过 78.5%。'];end
        caption(f,note);export(f,name);
    end
    function heat(ax,v,low,high)
        imagesc(ax,v,[low high]);axis(ax,'xy');axis(ax,'square');
        colormap(ax,palette);colorbar(ax);
        ax.XTick=1:7;ax.YTick=1:7;ax.XTickLabel=compose('%.1f',x);ax.YTickLabel=compose('%.1f',y);
        xlabel(ax,'规划余量 / m');ylabel(ax,'参考余量 / m');ax.TickLength=[0 0];
    end
    function caption(f,note)
        annotation(f,'textbox',[.08 .015 .86 .09],'String',note,'EdgeColor','none', ...
            'Color',[.34 .39 .45],'FontSize',12,'HorizontalAlignment','left','VerticalAlignment','middle');
    end
    function export(f,name)
        exportgraphics(f,fullfile(output,[name '.png']),'Resolution',200);
        exportgraphics(f,fullfile(output,[name '.pdf']),'ContentType','vector');
        savefig(f,fullfile(output,[name '.fig']));close(f);
    end
end
function color=pick_color(value,maximum)
    color=[.1 .15 .2];if maximum>0 && value/maximum>.65,color=[1 1 1];end
end

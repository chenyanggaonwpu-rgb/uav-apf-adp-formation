function plot_obstacle_sweep(outputDir)
% PLOT_OBSTACLE_SWEEP Academic PNG/PDF figures from saved, auditable rows.
    loaded=load(fullfile(outputDir,'experiment.mat'),'cfg'); cfg=loaded.cfg;
    rows=readtable(fullfile(outputDir,'trials.csv'),'TextType','string');
    summary=obstacle_sweep_summary(rows,cfg.combinations,cfg.seeds);
    writetable(summary,fullfile(outputDir,'summary.csv'));
    figDir=fullfile(outputDir,'figures'); if ~isfolder(figDir), mkdir(figDir); end
    full=height(rows)==height(cfg.jobs);
    if full, status=sprintf('每组 %d 个 seed',numel(cfg.seeds));
    else, status=sprintf('阶段性结果：%d / %d 次，样本尚未齐全',height(rows),height(cfg.jobs)); end
    context=sprintf('限速 %.1f m/s；规划净空 %.1f m；%s', ...
        cfg.baseParams.vmax,cfg.baseParams.pathClearance,status);
    counts=unique(summary.nObstacles); bands=unique(summary.corridorBand); gaps=unique(summary.marginObs);
    fixed=[counts(ceil(end/2)),bands(ceil(end/2)),gaps(ceil(end/2))];
    fields={'nObstacles','corridorBand','marginObs'};
    labels={'障碍物数量 / 个','航线两侧散布半宽 / m','最小边缘间距 / m'};
    titles={sprintf('散布半宽 %.0f m，最小间距 %.0f m',fixed(2),fixed(3)), ...
        sprintf('障碍物 %.0f 个，最小间距 %.0f m',fixed(1),fixed(3)), ...
        sprintf('障碍物 %.0f 个，散布半宽 %.0f m',fixed(1),fixed(2))};
    fig=new_figure([1320 500]); layout=tiledlayout(fig,1,3,'TileSpacing','compact','Padding','compact');
    for a=1:3
        ax=nexttile(layout); keep=true(height(summary),1);
        for b=1:3, if b~=a, keep=keep & summary.(fields{b})==fixed(b); end, end
        s=sortrows(summary(keep,:),fields{a});
        errorbar(ax,s.(fields{a}),s.successPct,s.successPct-s.ci95LowPct, ...
            s.ci95HighPct-s.successPct,'o-','Color',[0.1 0.18 0.26], ...
            'MarkerFaceColor','w','LineWidth',1.3,'MarkerSize',6,'CapSize',6);
        xlabel(ax,labels{a}); ylabel(ax,'飞行成功率 / %'); title(ax,titles{a},'FontWeight','normal');
        x=s.(fields{a}); xlim(ax,[min(x)-0.5 max(x)+0.5]); xticks(ax,x);
        ylim(ax,[0 103]); yticks(ax,0:20:100); style_axes(ax);
    end
    title(layout,{'三项环境参数与飞行成功率（其余两项固定）', ...
        [context '；误差棒：逐组 Wilson 95% 区间']},'FontWeight','normal','FontSize',13);
    save_figure(fig,figDir,'01_factor_effects');
    metrics={'successPct','overallSuccessPct','generationFailurePct','meanSuccessTime_s'};
    names={'02_flight_success_heatmaps','03_overall_success_heatmaps', ...
        '04_generation_failure_heatmaps','05_completion_time_heatmaps'};
    headings={'已生成场景中的飞行成功率 / %','全部计划试验的成功比例 / %', ...
        '整组障碍场景生成失败率 / %','成功样本平均完成时间 / s'};
    for m=1:numel(metrics)
        fig=new_figure([1380 860]); layout=tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');
        limits=[0 100];
        if m==4
            valid=summary.(metrics{m}); valid=valid(isfinite(valid));
            if isempty(valid), limits=[0 1]; else, limits=[floor(min(valid)) ceil(max(valid))+1]; end
        end
        for j=1:numel(gaps)
            ax=nexttile(layout); z=nan(numel(counts),numel(bands)); ns=zeros(size(z));
            for y=1:numel(counts)
                for x=1:numel(bands)
                    ix=summary.nObstacles==counts(y)&summary.corridorBand==bands(x)&summary.marginObs==gaps(j);
                    z(y,x)=summary.(metrics{m})(ix);
                    ns(y,x)=summary.generated(ix);
                end
            end
            h=imagesc(ax,1:numel(bands),1:numel(counts),z,limits);
            h.AlphaData=isfinite(z); ax.Color=[0.9 0.9 0.9]; ax.YDir='normal';
            xticks(ax,1:numel(bands)); xticklabels(ax,string(bands));
            yticks(ax,1:numel(counts)); yticklabels(ax,string(counts));
            xlabel(ax,'航线两侧散布半宽 / m'); ylabel(ax,'障碍物数量 / 个');
            title(ax,sprintf('最小边缘间距 = %g m',gaps(j)),'FontWeight','normal');
            clim(ax,limits); colormap(ax,academic_map()); colorbar(ax);
            for y=1:numel(counts)
                for x=1:numel(bands)
                    if isfinite(z(y,x))
                        color=[0.05 0.05 0.05];
                        if z(y,x)>(limits(1)+0.6*diff(limits)), color=[1 1 1]; end
                        text(ax,x,y,sprintf('%.1f',z(y,x)),'HorizontalAlignment','center', ...
                            'FontSize',11,'Color',color,'FontName','Times New Roman');
                    else
                        text(ax,x,y,'NA','HorizontalAlignment','center','FontSize',10);
                    end
                end
            end
            style_axes(ax); grid(ax,'off');
        end
        ax=nexttile(layout); axis(ax,'off');
        notes={sprintf('限速 %.1f m/s；规划净空 %.1f m',cfg.baseParams.vmax,cfg.baseParams.pathClearance); ...
            status;'每格为一个参数组合，无插值。'; ...
            '散布半宽 B 表示航线左右各 B 米。'; ...
            '飞行成功率分母：已生成且完成判定数。'; ...
            '整体比例分母：已执行的 seed 数。'; ...
            '生成失败：耗尽重试，不是单次拒绝。'; ...
            '有执行错误的成功率格子显示 NA。'; ...
            '样本数、95% 区间见 summary.csv。'};
        text(ax,0,0.95,notes,'Units','normalized','VerticalAlignment','top', ...
            'FontName','Microsoft YaHei','FontSize',10,'Interpreter','none');
        title(layout,headings{m},'FontWeight','normal','FontSize',15);
        save_figure(fig,figDir,names{m});
    end
    fig=new_figure([1060 740]); ax=axes(fig);
    scatter3(ax,summary.nObstacles,summary.corridorBand,summary.marginObs, ...
        75,summary.successPct,'filled','MarkerEdgeColor',[0.15 0.15 0.15]);
    xlabel(ax,labels{1}); ylabel(ax,labels{2}); zlabel(ax,labels{3});
    xticks(ax,counts); yticks(ax,bands); zticks(ax,gaps); clim(ax,[0 100]);
    colormap(ax,academic_map()); cb=colorbar(ax); cb.Label.String='飞行成功率 / %';
    view(ax,42,25); style_axes(ax);
    title(ax,{'三参数组合与成功率：每点为一个实测组合';context},'FontWeight','normal');
    save_figure(fig,figDir,'06_three_factor_scatter');
    % Interaction plot at every fixed minimum gap; intervals remain in CSV.
    fig=new_figure([1380 860]); layout=tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');
    palette=[0.05 0.05 0.05;0.20 0.32 0.43;0.36 0.43 0.50;0.48 0.48 0.48;0.65 0.65 0.65];
    markers={'o','s','^','d','v'};
    for j=1:numel(gaps)
        ax=nexttile(layout); hold(ax,'on');
        for b=1:numel(bands)
            s=sortrows(summary(summary.marginObs==gaps(j)&summary.corridorBand==bands(b),:),'nObstacles');
            plot(ax,s.nObstacles,s.successPct,'-','Marker',markers{1+mod(b-1,5)}, ...
                'Color',palette(1+mod(b-1,5),:),'LineWidth',1.2,'MarkerFaceColor','w');
        end
        xlabel(ax,labels{1}); ylabel(ax,'飞行成功率 / %'); xticks(ax,counts); ylim(ax,[0 103]); yticks(ax,0:20:100);
        title(ax,sprintf('最小边缘间距 = %g m',gaps(j)),'FontWeight','normal'); style_axes(ax);
    end
    legend(ax,compose('散布半宽 %g m',bands),'Location','best','FontSize',9);
    ax=nexttile(layout); axis(ax,'off');
    text(ax,0,0.9,{sprintf('限速 %.1f m/s；规划净空 %.1f m',cfg.baseParams.vmax,cfg.baseParams.pathClearance); ...
        status;'折线仅连接实测参数点。';'用于比较参数间的交互。';'不作因果或显著性结论。'; ...
        '95% 区间及分母见 summary.csv。'},'Units','normalized', ...
        'VerticalAlignment','top','FontName','Microsoft YaHei','FontSize',10);
    title(layout,'障碍物数量与散布宽度的联合作用','FontWeight','normal','FontSize',15);
    save_figure(fig,figDir,'07_interaction_curves');
end

function fig=new_figure(sz)
    fig=figure('Visible','off','Color','w','Position',[50 50 sz]);
    set(fig,'DefaultAxesFontName','Microsoft YaHei','DefaultTextFontName','Microsoft YaHei');
end
function style_axes(ax)
    ax.FontName='Microsoft YaHei'; ax.FontSize=10; ax.LineWidth=0.75;
    ax.Box='on'; ax.GridColor=[0.65 0.65 0.65]; ax.GridAlpha=0.2; grid(ax,'on');
end
function c=academic_map()
    c=[linspace(0.97,0.12,256).',linspace(0.98,0.25,256).',linspace(0.99,0.36,256).'];
end
function save_figure(fig,dir,name)
    cleanup=onCleanup(@() close(fig)); %#ok<NASGU>
    exportgraphics(fig,fullfile(dir,[name '.png']),'Resolution',220);
    exportgraphics(fig,fullfile(dir,[name '.pdf']),'ContentType','vector');
end

function plot_midpoint_sweep3d(studyDir,exportExample)
% PLOT_MIDPOINT_SWEEP3D Source-backed standalone research figures and example.
    if nargin<2,exportExample=true;end
    output=fullfile(studyDir,'figures'); if ~isfolder(output), mkdir(output); end
    rows=readtable(fullfile(studyDir,'adaptive_results.csv'),'Delimiter',',','TextType','string','VariableNamingRule','preserve');
    family=readtable(fullfile(studyDir,'family_summary.csv'),'Delimiter',',','TextType','string','VariableNamingRule','preserve');
    pair=readtable(fullfile(studyDir,'paired_results.csv'),'Delimiter',',','TextType','string','VariableNamingRule','preserve');
    base=readtable(fullfile(studyDir,'baseline_results.csv'),'Delimiter',',','TextType','string','VariableNamingRule','preserve');
    blue=[0 .42 .88]; grey=[.57 .61 .67]; orange=[.93 .48 .13]; dark=[.12 .14 .18];
    fig=new_figure(); ax=axes(fig); values=[family.baselinePercent,family.adaptivePercent];
    bars=bar(ax,values,.8); bars(1).FaceColor=grey; bars(2).FaceColor=blue; hold(ax,'on');
    for i=1:2
        if i==1, lower=family.baselineLow; upper=family.baselineHigh; count=family.baselineSuccess;
        else, lower=family.adaptiveLow; upper=family.adaptiveHigh; count=family.adaptiveSuccess; end
        errorbar(ax,bars(i).XEndPoints,values(:,i),values(:,i)-lower,upper-values(:,i), ...
            'Color',dark,'LineStyle','none','LineWidth',1,'CapSize',8,'HandleVisibility','off');
        for j=1:height(family)
            text(ax,bars(i).XEndPoints(j),upper(j)+2.8,sprintf('%d/%d\n%.1f%%',count(j),family.n(j),values(j,i)), ...
                'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',10,'Color',dark);
        end
    end
    xticks(ax,1:5); xticklabels(ax,{'All maps','Mixed solids','Underpass','One window','Two windows'});
    ylabel(ax,'Complete flight success / %'); ylim(ax,[0 120]); yticks(ax,0:20:100);
    title(ax,{'Same 1,000 historical maps: fixed layers vs midpoint fallback','95% Wilson intervals; all planned seeds included'});
    legend(ax,{'Fixed five layers','Midpoint fallback'},'Location','southoutside','Orientation','horizontal'); style(ax);
    save_chart(fig,'01_success_by_family'); writetable(family,fullfile(output,'01_success_by_family_data.csv'));
    states=["retained_success","rescued","lost_success","both_failed"];
    counts=zeros(1,4); for i=1:4, counts(i)=nnz(pair.transition==states(i)); end
    fig=new_figure(); ax=axes(fig); b=bar(ax,counts,.6,'FaceColor','flat');
    b.CData=[grey;blue;orange;[.78 .81 .85]];
    xticks(ax,1:4);xticklabels(ax,{'Success retained','Old failure rescued','Old success lost','Both failed'});
    ylabel(ax,'Number of maps'); ylim(ax,[0 max(counts)*1.18+10]);
    for i=1:4, text(ax,i,counts(i)+20,sprintf('%d',counts(i)),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',15); end
    title(ax,{'Paired outcomes on identical geometry','The original cohort contains 564 successes and 436 planning failures'});style(ax);
    save_chart(fig,'02_paired_outcomes');
    data=table(states',counts','VariableNames',{'transition','maps'});writetable(data,fullfile(output,'02_paired_outcomes_data.csv'));
    levels=unique(rows.initialLayerCount); counts=zeros(size(levels));
    for i=1:numel(levels),counts(i)=nnz(rows.initialLayerCount==levels(i));end
    fig=new_figure();ax=axes(fig);bar(ax,levels,counts,.65,'FaceColor',blue);
    xticks(ax,levels);xlabel(ax,'Number of working height layers in initial planning');ylabel(ax,'Number of tasks');
    ylim(ax,[0 max(counts)*1.18+10]);
    for i=1:numel(levels),text(ax,levels(i),counts(i)+8,sprintf('%d',counts(i)),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',15);end
    title(ax,{'Temporary height layers added only after fixed-layer planning fails','Five base layers; at most eight extra layers per planning call'});style(ax);
    save_chart(fig,'03_initial_layer_counts');
    data=table(levels,counts,'VariableNames',{'initialLayers','tasks'});writetable(data,fullfile(output,'03_initial_layer_counts_data.csv'));
    outcomes=["success","planning_failed","timeout","safety_unresolved","collision","boundary_violation","invalid_state","execution_error","replan_failed"];
    columns=zeros(2,numel(outcomes));
    for i=1:numel(outcomes),columns(:,i)=[nnz(base.reason==outcomes(i));nnz(rows.reason==outcomes(i))];end
    active=any(columns>0,1); columns=columns(:,active);outcomes=outcomes(active);
    colours=[blue;[.78 .81 .85];orange;[.70 .22 .45];[.52 .15 .35];[.43 .28 .54];[.46 .34 .16];[.24 .25 .29];[.66 .61 .38]];colours=colours(active,:);
    fig=new_figure();ax=axes(fig);bars=barh(ax,columns,'stacked');hold(ax,'on');
    for j=1:numel(bars),bars(j).FaceColor=colours(j,:);end
    for i=1:2
        left=0;
        for j=1:size(columns,2)
            if columns(i,j)>0
                if columns(i,j)>=80
                    text(ax,left+columns(i,j)/2,i,sprintf('%d',columns(i,j)), ...
                        'HorizontalAlignment','center','FontWeight','bold','FontSize',15,'Color',dark);
                end
                left=left+columns(i,j);
            end
        end
    end
    yticks(ax,1:2);yticklabels(ax,{'Fixed five layers','Midpoint fallback'});xlim(ax,[0 1000]);xlabel(ax,'Number of tasks / 1,000');
    title(ax,'Every outcome retained in the denominator');style(ax);
    legend(ax,strrep(cellstr(outcomes),'_',' '),'Location','southoutside','Orientation','horizontal');
    save_chart(fig,'04_outcome_breakdown');
    data=table(outcomes',columns(1,:)',columns(2,:)','VariableNames',{'outcome','baselineCount','adaptiveCount'});writetable(data,fullfile(output,'04_outcome_breakdown_data.csv'));
    % Predetermined illustration: smallest rescued two-window seed.
    chosen=sortrows(pair(pair.family=="staggered_windows" & pair.transition=="rescued",:),'seed');
    if ~isempty(chosen) && exportExample
        seed=chosen.seed(1); worker=ceil(seed/250);
        file=fullfile(studyDir,'workers',sprintf('worker_%02d',worker),'trials',sprintf('adaptive_%06d.mat',seed));
        data=load(file,'r'); r=data.r;
        example=fullfile(studyDir,'examples',sprintf('seed_%06d',seed));if ~isfolder(example),mkdir(example);end
        plot_result3d(r,example);render_3d_animation(r,fullfile(example,'flight_3d.mp4'));
        hs=r.plan.heightSelection;
        tableData=array2table(hs.sections,'VariableNames',{'probeX','probeY','heightLower','heightUpper'});
        tableData.decision=hs.decisions;writetable(tableData,fullfile(example,'height_intervals.csv'));
        traces=array2table([r.history.t(:),squeeze(r.history.state(3,:,:))'],'VariableNames', ...
            {'time','leaderZ','follower2Z','follower3Z','follower4Z','follower5Z'});
        writetable(traces,fullfile(example,'actual_heights.csv'));
        record=struct('seed',seed,'rule','Smallest rescued seed in the two-window family; selected after all outcomes recorded.', ...
            'sourceMat',file,'layers',hs.layers,'addedHeights',hs.addedHeights,'success',r.success);
        fid=fopen(fullfile(studyDir,'example_selection.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(record,PrettyPrint=true));fclose(fid);
    end
    fprintf('MIDPOINT_FIGURES_COMPLETE count=4\n');
    function fig=new_figure()
        fig=figure('Visible','off','Color','w','Position',[80 80 1450 820]);
    end
    function style(ax)
        set(ax,'FontName','Arial','FontSize',12,'LineWidth',1,'Box','off','XColor',dark,'YColor',dark);
        grid(ax,'on');ax.GridAlpha=.10;ax.Layer='top';
    end
    function save_chart(fig,name)
        exportgraphics(fig,fullfile(output,[name '.png']),'Resolution',180);
        exportgraphics(fig,fullfile(output,[name '.pdf']),'ContentType','vector');
        savefig(fig,fullfile(output,[name '.fig']));close(fig);
    end
end

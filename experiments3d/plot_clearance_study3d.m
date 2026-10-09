function plot_clearance_study3d(studyDir)
% PLOT_CLEARANCE_STUDY3D Source-backed PNG/PDF/FIG research figures.
    output=fullfile(studyDir,'figures'); if ~isfolder(output), mkdir(output); end
    set(groot,'defaultAxesFontName','Arial','defaultAxesFontSize',11);
    set(groot,'defaultTextFontName','Arial');
    blue=[0.05 0.38 0.78]; green=[0.02 0.57 0.43]; red=[0.78 0.22 0.19];
    screen=readtable(fullfile(studyDir,'screen_results.csv'),'Delimiter',',','VariableNamingRule','preserve','TextType','string');
    zValues=unique(screen.planZ(screen.enabled==1)); hValues=unique(screen.pathXY(screen.enabled==1));
    dValues=unique(screen.refXY(screen.enabled==1));
    fig=figure('Visible','off','Color','w','Position',[20 20 1500 900]);
    layout=tiledlayout(fig,2,4,'TileSpacing','compact','Padding','compact'); chartRows=struct([]);
    for k=1:numel(zValues)
        ax=nexttile(layout); matrix=nan(numel(hValues),numel(dValues));
        for h=1:numel(hValues)
            for d=1:numel(dValues)
                sub=screen(screen.enabled==1 & screen.pathXY==hValues(h) & ...
                    screen.refXY==dValues(d) & screen.planZ==zValues(k),:);
                matrix(h,d)=100*mean(sub.success);
                chartRows=[chartRows;struct('pathXY',hValues(h),'refXY',dValues(d), ...
                    'z',zValues(k),'n',height(sub),'success',sum(sub.success), ...
                    'successPercent',matrix(h,d))]; %#ok<AGROW>
            end
        end
        imagesc(ax,1:numel(dValues),1:numel(hValues),matrix); set(ax,'YDir','normal'); clim(ax,[0 100]);
        xticks(ax,1:numel(dValues)); xticklabels(ax,compose('%g',dValues));
        yticks(ax,1:numel(hValues)); yticklabels(ax,compose('%g',hValues));
        xlabel(ax,'Reference XY margin / m'); ylabel(ax,'Planning XY margin / m');
        title(ax,sprintf('Vertical margin %.2g m',zValues(k)));
        for h=1:numel(hValues)
            for d=1:numel(dValues)
                color='k'; if matrix(h,d)>60, color='w'; end
                text(ax,d,h,sprintf('%.1f',matrix(h,d)),'HorizontalAlignment','center','Color',color,'FontSize',9);
            end
        end
    end
    colormap(fig,[linspace(0.95,blue(1),128)',linspace(0.97,blue(2),128)',linspace(1,blue(3),128)']);
    ax=nexttile(layout); axis(ax,'off'); clim(ax,[0 100]); colorbar(ax);
    text(ax,0,0.7,{'End-to-end success / %','8 shared maps per setting', ...
        'Exploratory screening only','Hard obstacle distance: 0.6 m','Fixed five height layers'},'FontSize',12);
    title(layout,'Global clearance screening'); save_chart(fig,'01_global_screen');
    writetable(struct2table(chartRows),fullfile(output,'01_global_screen_data.csv'));

    if isfile(fullfile(studyDir,'holdout_results.csv'))
        holdout=readtable(fullfile(studyDir,'holdout_results.csv'),'Delimiter',',','VariableNamingRule','preserve','TextType','string');
        ids=unique(holdout.configId,'stable'); families=["mixed","underpass","window","staggered_windows"];
        values=zeros(4,numel(ids)); lower=values; upper=values; rows=struct([]);
        for i=1:numel(ids)
            for f=1:4
                sub=holdout(holdout.configId==ids(i) & holdout.family==families(f),:);
                n=height(sub); g=sum(sub.success); [lo,hi]=wilson(g,n);
                values(f,i)=100*g/n; lower(f,i)=100*lo; upper(f,i)=100*hi;
                rows=[rows;struct('configId',ids(i),'family',families(f),'n',n,'success',g, ...
                    'successPercent',values(f,i),'wilsonLowerPercent',lower(f,i),'wilsonUpperPercent',upper(f,i))]; %#ok<AGROW>
            end
        end
        fig=figure('Visible','off','Color','w','Position',[20 20 1250 650]); ax=axes(fig);
        bars=bar(ax,values,'grouped'); hold(ax,'on');
        for i=1:numel(ids)
            bars(i).FaceColor=blue; if i==2, bars(i).FaceColor=green; end
            errorbar(ax,bars(i).XEndPoints,values(:,i),values(:,i)-lower(:,i), ...
                upper(:,i)-values(:,i),'k.','LineWidth',1);
        end
        xticklabels(ax,{'Mixed solids','Underpass','Single window','Two windows'});
        ylabel(ax,'End-to-end success / %'); ylim(ax,[0 105]); legend(ax,cellstr(ids),'Location','southoutside','Orientation','horizontal');
        title(ax,{'Independent maps: seeds 1001-2000','250 maps per family and setting; Wilson 95% intervals'});
        style(ax); save_chart(fig,'02_holdout_success'); writetable(struct2table(rows),fullfile(output,'02_holdout_success_data.csv'));

        reasons=unique(holdout.reason); counts=zeros(numel(ids),numel(reasons));
        for i=1:numel(ids), for j=1:numel(reasons), counts(i,j)=nnz(holdout.configId==ids(i) & holdout.reason==reasons(j)); end; end
        fig=figure('Visible','off','Color','w','Position',[20 20 1100 550]); ax=axes(fig);
        outcomeBars=barh(ax,counts,'stacked'); outcomeRows=struct([]);
        for j=1:numel(reasons)
            outcomeBars(j).FaceColor=[0.38 0.42 0.47];
            if reasons(j)=="success", outcomeBars(j).FaceColor=green; end
            for i=1:numel(ids)
                n=nnz(holdout.configId==ids(i));
                outcomeRows=[outcomeRows;struct('configId',ids(i),'reason',reasons(j), ...
                    'n',n,'count',counts(i,j),'percent',100*counts(i,j)/n)]; %#ok<AGROW>
                if counts(i,j)>0
                    x=sum(counts(i,1:j))-counts(i,j)/2;
                    text(ax,x,i,sprintf('%d',counts(i,j)),'HorizontalAlignment','center','Color','w','FontWeight','bold');
                end
            end
        end
        yticklabels(ax,cellstr(ids)); xlabel(ax,'Scheduled maps');
        legend(ax,cellstr(strrep(reasons,'_',' ')),'Location','southoutside','Orientation','horizontal');
        title(ax,'Independent validation outcomes'); style(ax); save_chart(fig,'03_holdout_outcomes');
        writetable(struct2table(outcomeRows),fullfile(output,'03_holdout_outcomes_data.csv'));
        % True height synchronization: compare equal successful maps, then
        % show the candidate's newly rescued cohort separately.
        control=holdout(holdout.configId=="baseline",:); candidate=holdout(holdout.configId~="baseline",:);
        sharedSeeds=intersect(control.seed(control.success==1),candidate.seed(candidate.success==1));
        a=control(ismember(control.seed,sharedSeeds),:); b=candidate(ismember(candidate.seed,sharedSeeds),:);
        fig=figure('Visible','off','Color','w','Position',[20 20 1300 550]); layout=tiledlayout(fig,1,2);
        ax=nexttile(layout); hold(ax,'on');
        stairs(ax,sort(a.followerHeightMax),(1:height(a))/height(a),'Color',blue,'LineWidth',2);
        stairs(ax,sort(b.followerHeightMax),(1:height(b))/height(b),'Color',green,'LineWidth',2);
        xlabel(ax,'Per-flight maximum true follower-leader height difference / m'); ylabel(ax,'Empirical cumulative fraction');
        legend(ax,{'Baseline','Candidate'},'Location','southeast'); title(ax,sprintf('%d shared successful maps',numel(sharedSeeds))); style(ax);
        ax=nexttile(layout); rescued=candidate(candidate.success==1 & ~ismember(candidate.seed,sharedSeeds),:);
        labels=[repmat("Shared success",height(b),1);repmat("Rescued maps",height(rescued),1)];
        boxchart(ax,categorical(labels),[b.followerHeightMax;rescued.followerHeightMax]);
        ylabel(ax,'Per-flight maximum true height difference / m'); title(ax,'Candidate cohorts shown separately'); style(ax);
        title(layout,'Height synchronization during flight'); save_chart(fig,'08_height_synchronization');
        data=holdout(:,{'configId','seed','success','followerHeightRms','followerHeightMax'});
        data.sharedSuccess=ismember(data.seed,sharedSeeds); writetable(data,fullfile(output,'08_height_synchronization_data.csv'));
    end
    if isfile(fullfile(studyDir,'boundary_results.csv'))
        data=readtable(fullfile(studyDir,'boundary_results.csv'),'Delimiter',',','VariableNamingRule','preserve','TextType','string');
        ids=unique(data.configId,'stable'); fig=figure('Visible','off','Color','w','Position',[20 20 1300 550]);
        layout=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact'); rows=struct([]);
        for panel=1:2
            ax=nexttile(layout); hold(ax,'on');
            if panel==1, field='noiseStd'; sub=data(data.disturbanceMax==0,:); else, field='disturbanceMax'; sub=data(data.noiseStd==0,:); end
            xs=unique(sub.(field));
            for i=1:numel(ids)
                ys=zeros(size(xs));
                for j=1:numel(xs)
                    part=sub(sub.configId==ids(i) & sub.(field)==xs(j),:); ys(j)=100*mean(part.success);
                    rows=[rows;struct('configId',ids(i),'axis',string(field),'strength',xs(j), ...
                        'n',height(part),'success',sum(part.success),'successPercent',ys(j))]; %#ok<AGROW>
                end
                color=blue; if ids(i)~="baseline", color=green; end
                plot(ax,xs,ys,'-o','Color',color,'LineWidth',2,'MarkerSize',5);
            end
            ylim(ax,[0 105]); ylabel(ax,'End-to-end success / %');
            if panel==1, xlabel(ax,'Position-noise standard deviation / m'); else, xlabel(ax,'Acceleration disturbance bound / m s^{-2}'); end
            legend(ax,cellstr(ids),'Location','southwest'); style(ax);
        end
        title(layout,'Observed disturbance boundary: 8 maps, 2 random repeats per setting');
        save_chart(fig,'04_disturbance_boundary'); writetable(struct2table(rows),fullfile(output,'04_disturbance_boundary_data.csv'));
    end
    if isfile(fullfile(studyDir,'geometry_results.csv'))
        data=readtable(fullfile(studyDir,'geometry_results.csv'),'Delimiter',',','VariableNamingRule','preserve','TextType','string');
        widths=[1 1.25 1.5 2 3 4 6 8 10]; centers=[10 11.25 12.5 13.75 15];
        ids=unique(data.configId,'stable'); policies=["fixed5","add_window_center"];
        fig=figure('Visible','off','Color','w','Position',[20 20 1450 850]); layout=tiledlayout(fig,numel(ids),2,'TileSpacing','compact','Padding','compact');
        for i=1:numel(ids)
            for p=1:2
                ax=nexttile(layout); matrix=zeros(numel(centers),numel(widths));
                sub=data(data.configId==ids(i) & data.layerPolicy==policies(p),:);
                for j=1:height(sub)
                    position=sub.seed(j)-900001; w=floor(position/numel(centers))+1; c=mod(position,numel(centers))+1;
                    matrix(c,w)=sub.success(j);
                    if ~sub.success(j) && sub.cycles(j)>0, matrix(c,w)=2; end
                end
                imagesc(ax,matrix); set(ax,'YDir','normal'); clim(ax,[0 2]);
                xticks(ax,1:numel(widths)); xticklabels(ax,compose('%g',widths));
                yticks(ax,1:numel(centers)); yticklabels(ax,compose('%g',centers));
                xlabel(ax,'Vertical opening height / m'); ylabel(ax,'Opening center / m');
                title(ax,sprintf('%s | %s',ids(i),strrep(policies(p),'_',' ')),'Interpreter','none');
                for w=1:numel(widths), for c=1:numel(centers)
                    label='Plan'; if matrix(c,w)==1, label='Pass'; elseif matrix(c,w)==2, label='Flight'; end
                    text(ax,w,c,label,'HorizontalAlignment','center','FontSize',8,'Color','w');
                end; end
            end
        end
        colormap(fig,[0.38 0.42 0.47;0.00 0.48 0.37;0.75 0.30 0.10]);
        title(layout,{'Controlled aperture boundary: outcomes, not population rates', ...
            'Plan = no initial route | Flight = began and failed | Pass = success'});
        save_chart(fig,'05_aperture_boundary');
        position=data.seed-900001;
        data.verticalOpeningHeight=reshape(widths(floor(position/numel(centers))+1),[],1);
        data.openingCenter=reshape(centers(mod(position,numel(centers))+1),[],1);
        writetable(data,fullfile(output,'05_aperture_boundary_data.csv'));
    end
    if isfile(fullfile(studyDir,'timing_results.csv'))
        data=readtable(fullfile(studyDir,'timing_results.csv'),'Delimiter',',','VariableNamingRule','preserve','TextType','string'); ids=unique(data.configId,'stable');
        fig=figure('Visible','off','Color','w','Position',[20 20 1250 550]); layout=tiledlayout(fig,1,2);
        ax=nexttile(layout); boxchart(ax,categorical(data.configId),data.initialPlanSeconds); ylabel(ax,'Initial planning / s'); style(ax);
        ax=nexttile(layout); boxchart(ax,categorical(data.configId),data.cycleP99ms); yline(ax,50,'--','50 ms budget'); ylabel(ax,'Per-flight cycle P99 / ms'); style(ax);
        title(layout,'Isolated timing: initial planning and control cycles reported separately'); save_chart(fig,'06_isolated_timing');
        writetable(data,fullfile(output,'06_isolated_timing_data.csv'));
    end
    if isfile(fullfile(studyDir,'ablation_results.csv'))
        data=readtable(fullfile(studyDir,'ablation_results.csv'),'Delimiter',',','VariableNamingRule','preserve','TextType','string'); ids=unique(data.configId,'stable');
        values=zeros(numel(ids),1); ablationRows=struct([]);
        for i=1:numel(ids)
            part=data(data.configId==ids(i),:); values(i)=100*mean(part.success);
            ablationRows=[ablationRows;struct('configId',ids(i),'n',height(part),'success',sum(part.success), ...
                'successPercent',values(i),'pathXY',part.pathXY(1),'refXY',part.refXY(1), ...
                'planZ',part.planZ(1),'refZ',part.refZ(1),'enabled',part.enabled(1))]; %#ok<AGROW>
        end
        fig=figure('Visible','off','Color','w','Position',[20 20 1100 600]); ax=axes(fig);
        barh(ax,values,'FaceColor',blue); yticks(ax,1:numel(ids)); yticklabels(ax,cellstr(strrep(ids,'_',' ')));
        xlabel(ax,'End-to-end success / %'); xlim(ax,[0 105]); title(ax,'Clearance ablation: 40 shared maps per setting'); style(ax); save_chart(fig,'07_clearance_ablation');
        writetable(struct2table(ablationRows),fullfile(output,'07_clearance_ablation_data.csv'));
    end

    function save_chart(fig,name)
        exportgraphics(fig,fullfile(output,[name '.png']),'Resolution',180);
        exportgraphics(fig,fullfile(output,[name '.pdf']),'ContentType','vector');
        savefig(fig,fullfile(output,[name '.fig'])); close(fig);
    end
end
function style(ax)
    grid(ax,'on'); ax.GridAlpha=0.12; ax.Box='off'; ax.TickLabelInterpreter='none';
end
function [lo,hi]=wilson(g,n)
    z=1.959963984540054; p=g/n; scale=1+z^2/n;
    center=(p+z^2/(2*n))/scale; radius=z*sqrt(p*(1-p)/n+z^2/(4*n^2))/scale;
    lo=max(0,center-radius); hi=min(1,center+radius);
end

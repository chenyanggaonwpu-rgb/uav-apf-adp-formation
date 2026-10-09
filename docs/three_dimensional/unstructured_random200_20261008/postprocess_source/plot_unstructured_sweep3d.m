function plot_unstructured_sweep3d(studyDir,exportVideo,exportExamples)
% PLOT_UNSTRUCTURED_SWEEP3D Source-backed scientific figures; no resimulation.
    if nargin<2,exportVideo=true;end
    if nargin<3,exportExamples=true;end
    root=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(root,'src3d'));
    options=detectImportOptions(fullfile(studyDir,'all_results.csv'),'Delimiter',',','VariableNamingRule','preserve');
    options=setvartype(options,{'reason','planStatus','fixedPlanStatus','addedHeights','mapHash'},'string');
    rows=readtable(fullfile(studyDir,'all_results.csv'),options);
    bins=readtable(fullfile(studyDir,'success_by_count.csv'),'Delimiter',',','TextType','string','VariableNamingRule','preserve');
    catalog=readtable(fullfile(studyDir,'obstacle_catalog.csv'),'Delimiter',',','TextType','string','VariableNamingRule','preserve');
    timing=readtable(fullfile(studyDir,'timing_by_seed.csv'),'Delimiter',',','TextType','string','VariableNamingRule','preserve');
    saved=load(fullfile(studyDir,'protocol.mat'),'protocol');protocol=saved.protocol;assert(height(rows)==200);
    output=fullfile(studyDir,'figures');if ~isfolder(output),mkdir(output);end
    blue=[.05 .38 .86];orange=[.86 .45 .17];grey=[.62 .66 .72];dark=[.12 .15 .21];
    expected=["success","invalid_scene","planning_failed","timeout","collision","safety_unresolved","replan_failed","boundary_violation","invalid_state"];
    states=[expected, setdiff(unique(rows.reason)',expected,'stable')];counts=zeros(size(states));
    for k=1:numel(states),counts(k)=nnz(rows.reason==states(k));end
    active=counts>0;states=states(active);counts=counts(active);
    labels=strrep(cellstr(states),'_',' '); labels(strcmp(states,'success'))={'Complete success'};
    labels(strcmp(states,'invalid_scene'))={'Unsafe initial formation'};
    labels(strcmp(states,'planning_failed'))={'Initial planning failed'};
    fig=new_figure();ax=axes(fig);bars=barh(ax,counts,.6,'FaceColor','flat');bars.CData=repmat(grey,numel(counts),1);
    bars.CData(states=="success",:)=blue;bars.CData(states~="success" & states~="invalid_scene",:)=repmat(orange,nnz(states~="success" & states~="invalid_scene"),1);
    set(ax,'YDir','reverse');yticks(ax,1:numel(states));yticklabels(ax,labels);xlim(ax,[0 max(counts)*1.30+1]);
    for k=1:numel(states),text(ax,counts(k)+1,k,sprintf('%d / 200   (%.1f%%)',counts(k),100*counts(k)/200),'FontSize',13,'Color',dark);end
    ax.Position=[.25 .14 .69 .74];
    xlabel(ax,'Maps');title(ax,{'Outcomes on 200 unconditioned random maps','Seeds 2001-2200; unsafe starts and every failure retained'});style(ax);save_chart(fig,'01_outcomes');
    writetable(table(states',counts',100*counts'/200,'VariableNames',{'outcome','maps','percent'}),fullfile(output,'01_outcomes_data.csv'));
    fig=new_figure();ax=axes(fig);bar(ax,bins.percent,.55,'FaceColor',blue);hold(ax,'on');
    errorbar(ax,1:height(bins),bins.percent, bins.percent-100*bins.wilsonLow,100*bins.wilsonHigh-bins.percent, ...
        'Color',dark,'LineStyle','none','LineWidth',1.2,'CapSize',10);
    for k=1:height(bins)
        text(ax,k,100*bins.wilsonHigh(k)+3,sprintf('%d/%d\n%.1f%%',bins.success(k),bins.n(k),bins.percent(k)), ...
            'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',13,'Color',dark);
    end
    xticks(ax,1:height(bins));xticklabels(ax,bins.countRange);ylim(ax,[0 119]);yticks(ax,0:20:100);
    xlabel(ax,'Random obstacle count');ylabel(ax,'Complete success / all maps in bin (%)');
    title(ax,{'Success by obstacle count','95% Wilson intervals; bins include unsafe starts'});style(ax);save_chart(fig,'02_success_by_count');
    writetable(bins,fullfile(output,'02_success_by_count_data.csv'));
    fig=new_figure();layout=tiledlayout(fig,2,2,'TileSpacing','loose','Padding','compact');
    ax=nexttile(layout);histogram(ax,rows.obstacleCount,.5:1:40.5,'FaceColor',blue,'EdgeColor','none');
    xlim(ax,[0 41]);xlabel(ax,'Obstacles / map');ylabel(ax,'Maps');title(ax,'Sampled obstacle counts');style(ax);
    types=["sphere","cylinder","box"];typeCounts=zeros(1,3);colours=[blue;orange;grey];
    for k=1:3,typeCounts(k)=nnz(catalog.type==types(k));end
    ax=nexttile(layout);bars=bar(ax,typeCounts,.55,'FaceColor','flat');bars.CData=colours;
    xticks(ax,1:3);xticklabels(ax,{'Sphere','Cylinder','Box'});ylabel(ax,'Solids across 200 maps');ylim(ax,[0 max(typeCounts)*1.22]);
    for k=1:3,text(ax,k,typeCounts(k)+20,sprintf('%d',typeCounts(k)),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',12);end
    title(ax,'Equal-probability type sampling');style(ax);
    ax=nexttile(layout);histogram(ax,catalog.z,0:2:30,'FaceColor',blue,'EdgeColor','none');
    xlim(ax,[0 30]);xlabel(ax,'Centre height (m)');ylabel(ax,'Solids');title(ax,'Height distribution after containment');style(ax);
    ax=nexttile(layout);hold(ax,'on');
    for k=1:3
        use=catalog.type==types(k);scatter(ax,catalog.sizeZ(use),catalog.z(use),12,colours(k,:),'filled','MarkerFaceAlpha',.23);
    end
    xlim(ax,[0 25]);ylim(ax,[0 30]);xlabel(ax,'Vertical extent (m)');ylabel(ax,'Centre height (m)');
    title(ax,'Sizes and heights are boundary constrained');style(ax);
    legend(ax,{'Sphere','Cylinder','Box'},'Location','southoutside','Orientation','horizontal');
    title(layout,{'Random input distribution','All generated solids; no endpoint reservation or reachability screening'},'FontSize',17,'Color',dark);
    save_chart(fig,'03_random_input_distribution');writetable(catalog,fullfile(output,'03_random_input_distribution_data.csv'));
    writetable(rows(:,{'seed','obstacleCount','sphereCount','cylinderCount','boxCount','nominalVolumeFraction'}),fullfile(output,'03_map_distribution_data.csv'));
    fig=new_figure();layout=tiledlayout(fig,1,2,'TileSpacing','loose','Padding','compact');
    ax=nexttile(layout);finite=isfinite(timing.initialPlanSeconds);
    histogram(ax,timing.initialPlanSeconds(finite),'FaceColor',blue,'EdgeColor','none');
    xlabel(ax,'Initial planning time (s)');ylabel(ax,'Planning calls');title(ax,sprintf('Initial planning (%d calls)',nnz(finite)));style(ax);
    ax=nexttile(layout);finite=isfinite(timing.p99CycleMs);scatter(ax,rows.obstacleCount(finite),timing.p99CycleMs(finite),40,blue,'filled');hold(ax,'on');
    yline(ax,1000*protocol.p.dt,'--','50 ms simulation step','Color',dark,'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom');
    ylim(ax,[0 max(55,1.10*max(timing.p99CycleMs(finite)))]);
    xlabel(ax,'Obstacles / map');ylabel(ax,'P99 online cycle time per flight (ms)');
    title(ax,sprintf('Measured flights (%d histories)',nnz(finite)));style(ax);
    title(layout,{'Recorded computation time','Four concurrent MATLAB processes; not a hard real-time test'},'FontSize',17,'Color',dark);
    save_chart(fig,'04_computation_time');timing.obstacleCount=rows.obstacleCount;writetable(timing,fullfile(output,'04_computation_time_data.csv'));
    if ~exportExamples
        fprintf('UNSTRUCTURED_CHARTS_ONLY_EXPORTED charts=4\n');return;
    end
    % Predetermined examples. Failures are preserved alongside the success.
    candidates=sortrows(rows(rows.success==1,:),'seed');examples=struct([]);
    if ~isempty(candidates),examples=[examples;export_case(candidates.seed(1),'smallest_success',exportVideo)];end %#ok<AGROW>
    candidates=sortrows(rows(rows.success==0 & rows.cycles>0,:),'seed');
    if ~isempty(candidates),examples=[examples;export_case(candidates.seed(1),'smallest_failure_with_history',false)];end %#ok<AGROW>
    candidates=sortrows(rows(rows.reason=="planning_failed",:),'seed');
    if ~isempty(candidates),examples=[examples;export_case(candidates.seed(1),'smallest_planning_failure',false)];end %#ok<AGROW>
    if ~isempty(examples)
        fid=fopen(fullfile(studyDir,'example_manifest.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(examples,PrettyPrint=true));fclose(fid);
    end
    fprintf('UNSTRUCTURED_FIGURES_EXPORTED charts=4 examples=%d\n',numel(examples));
    function fig=new_figure()
        fig=figure('Visible','off','Color','w','Position',[100 100 1400 880]);
    end
    function style(ax)
        ax.FontName='Arial';ax.FontSize=13;ax.XColor=dark;ax.YColor=dark;ax.GridColor=[.87 .89 .92];ax.GridAlpha=.5;
        ax.LineWidth=.8;box(ax,'off');grid(ax,'on');ax.Layer='top';
    end
    function save_chart(fig,name)
        exportgraphics(fig,fullfile(output,[name '.png']),'Resolution',180);
        exportgraphics(fig,fullfile(output,[name '.pdf']),'ContentType','vector');
        savefig(fig,fullfile(output,[name '.fig']));close(fig);
    end
    function record=export_case(seed,rule,video)
        worker=find(seed>=protocol.ranges(:,1) & seed<=protocol.ranges(:,2));
        rawFile=fullfile(studyDir,'workers',sprintf('worker_%02d',worker),'trials',sprintf('random_%06d.mat',seed));
        d=load(rawFile,'r','q');r=d.r;q=d.q;
        directory=fullfile(studyDir,'examples',sprintf('seed_%06d',seed));if ~isfolder(directory),mkdir(directory);end
        actualVideo=false;
        if isfield(r.history,'t') && ~isempty(r.history.t)
            plot_result3d(r,directory);
            data=table(r.history.t,'VariableNames',{'time'});
            for k=1:q.nUAV,data.(sprintf('heightUAV%d',k))=squeeze(r.history.state(3,k,:));end
            writetable(data,fullfile(directory,'actual_heights.csv'));
            if video,render_3d_animation(r,fullfile(directory,'flight_3d.mp4'));actualVideo=true;end
        else
            fig=new_figure();ax=axes(fig);hold(ax,'on');draw_obstacles3d(ax,r.scene.obstacles,true,.30);
            state=init_formation3d(q);scatter3(ax,state(1,:),state(2,:),state(3,:),80,blue,'filled');
            scatter3(ax,q.target(1),q.target(2),q.target(3),140,orange,'filled','p');
            xlim(ax,q.workspace(:,1)');ylim(ax,q.workspace(:,2)');zlim(ax,q.workspace(:,3)');
            xlabel(ax,'x (m)');ylabel(ax,'y (m)');zlabel(ax,'z (m)');view(ax,35,25);axis(ax,'equal');style(ax);
            title(ax,sprintf('Seed %d | %s | no flight history',seed,r.reason),'Interpreter','none');
            exportgraphics(fig,fullfile(directory,'map_3d.png'),'Resolution',180);savefig(fig,fullfile(directory,'map_3d.fig'));close(fig);
        end
        if isfield(r.plan,'heightSelection')
            hs=r.plan.heightSelection;
            data=array2table(hs.sections,'VariableNames',{'probeX','probeY','heightLower','heightUpper'});data.decision=hs.decisions;
            writetable(data,fullfile(directory,'height_intervals.csv'));
        end
        record=struct('seed',seed,'rule',rule,'reason',r.reason,'obstacles',numel(r.scene.obstacles), ...
            'path',strrep(extractAfter(string(directory),strlength(string(studyDir))+1),'\','/'),'mp4',actualVideo, ...
            'videoWidth',nan,'videoHeight',nan,'videoFps',nan,'videoDuration',nan);
        if actualVideo
            reader=VideoReader(fullfile(directory,'flight_3d.mp4'));qa=fullfile(directory,'video_qa');if ~isfolder(qa),mkdir(qa);end
            for k=1:3
                fractions=[.05 .5 .95];reader.CurrentTime=fractions(k)*reader.Duration;frame=readFrame(reader);imwrite(frame,fullfile(qa,sprintf('frame_%d.png',k)));
            end
            record.videoWidth=reader.Width;record.videoHeight=reader.Height;record.videoFps=reader.FrameRate;record.videoDuration=reader.Duration;
        end
    end
end

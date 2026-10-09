function media=export_seed_media3d(seeds,studyDir,outputDir)
% EXPORT_SEED_MEDIA3D Export saved seed trajectories; never rerun a flight.
% export_seed_media3d([1 2 3 168]) exports the four successful examples.
% A seed without flight history produces a scene PNG and editable FIG.
    root=fileparts(fileparts(mfilename('fullpath')));
    if nargin<2 || isempty(studyDir)
        studyDir=fullfile(root,'results','seed_sweep3d_v2.1.0_20261005_113155_970');
    end
    if nargin<3 || isempty(outputDir)
        outputDir=fullfile(root,'交付文件','新障碍物_固定五层_1000seed_20261005','seed_media');
    end
    validateattributes(seeds,{'numeric'},{'vector','integer','positive','finite'});
    seeds=unique(seeds,'stable');
    saved=load(fullfile(studyDir,'protocol.mat'),'protocol');
    protocol=saved.protocol;
    assert(all(ismember(seeds,protocol.seeds)),'UAV3D:MediaSeeds', ...
        'Requested seeds must belong to the saved protocol.');
    if ~isfolder(outputDir), mkdir(outputDir); end
    oldPath=path; pathCleanup=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(studyDir,'source_snapshot','src3d'), ...
        fullfile(studyDir,'source_snapshot','config'));
    records=struct([]);
    for seed=seeds(:)'
        worker=find(seed>=protocol.ranges(:,1) & seed<=protocol.ranges(:,2));
        assert(isscalar(worker));
        source=fullfile(studyDir,'workers',sprintf('worker_%02d',worker), ...
            'trials',sprintf('C_random_%06d.mat',seed));
        data=load(source,'q','r','scene','row');
        assert(data.q.seed==seed && data.row.seed==seed && data.scene.seed==seed);
        family=string(data.scene.family); reason=string(data.row.reason);
        stem=sprintf('seed_%06d_%s',seed,family);
        started=tic;
        record=struct('seed',seed,'family',family,'reason',reason, ...
            'mediaType',"scene",'relativeFile',"",'sourceMat',string(source), ...
            'simulationSeconds',data.row.time,'playbackSeconds',nan, ...
            'frameRate',nan,'frameCount',0,'width',0,'height',0, ...
            'renderWallSeconds',nan);
        hasFlight=~isempty(data.r) && isfield(data.r.history,'state') && ...
            isfield(data.r.history,'t') && ~isempty(data.r.history.t);
        if hasFlight
            animation=data.r;
            % Change the video label only; the saved scene and states are untouched.
            animation.scene.id=sprintf('seed %d | %s',seed,family);
            outputFile=fullfile(outputDir,[stem '.mp4']);
            expectedFrames=numel(unique([1:4:numel(animation.history.t), ...
                numel(animation.history.t)]));
            reuse=false;
            if isfile(outputFile)
                try
                    existing=VideoReader(outputFile);
                    reuse=abs(existing.FrameRate-12)<0.01 && ...
                        abs(existing.Duration-expectedFrames/12)<1/12+1e-6;
                    clear existing;
                catch
                    reuse=false;
                end
            end
            if ~reuse, render_3d_animation(animation,outputFile); end
            video=VideoReader(outputFile);
            % getframe pixel dimensions vary with Windows display scaling.
            assert(abs(video.FrameRate-12)<0.01 && video.Width>=1280 && video.Height>=720);
            assert(abs(video.Width/video.Height-16/9)<0.01);
            assert(abs(video.Duration-expectedFrames/12)<1/12+1e-6);
            video.CurrentTime=min(video.Duration/2,max(0,video.Duration-1/video.FrameRate));
            imwrite(readFrame(video),fullfile(outputDir,[stem '_preview.png']));
            record.mediaType="mp4"; record.relativeFile=string([stem '.mp4']);
            record.playbackSeconds=video.Duration; record.frameRate=video.FrameRate;
            record.frameCount=expectedFrames; record.width=video.Width; record.height=video.Height;
        else
            outputFile=fullfile(outputDir,[stem '_planning_failed.png']);
            renderFailedScene(data,seed,outputFile);
            record.relativeFile=string([stem '_planning_failed.png']);
        end
        record.renderWallSeconds=toc(started);
        records=[records;record]; %#ok<AGROW>
        fprintf('SEED_MEDIA seed=%d family=%s reason=%s file=%s wall=%.1f s\n', ...
            seed,family,reason,outputFile,record.renderWallSeconds);
    end
    media=struct2table(records);
    indexFile=fullfile(outputDir,'media_index.csv');
    combined=media;
    if isfile(indexFile)
        previous=readtable(indexFile,'TextType','string');
        previous=previous(~ismember(previous.seed,seeds),:);
        combined=[previous;media];
    end
    writetable(sortrows(combined,'seed'),indexFile);
end

function renderFailedScene(data,seed,outputFile)
    q=data.q; scene=data.scene;
    fig=figure('Visible','off','Color','w','Position',[50 50 1400 780]);
    cleanup=onCleanup(@() close(fig)); %#ok<NASGU>
    layout=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
    ax=nexttile(layout); hold(ax,'on'); grid(ax,'on');
    draw_obstacles3d(ax,scene.obstacles,true,0.38);
    if ~isempty(data.r), initial=data.r.state(1:3,:);
    else, state=init_formation3d(q); initial=state(1:3,:); end
    scatter3(ax,initial(1,:),initial(2,:),initial(3,:),65,'filled');
    scatter3(ax,q.target(1),q.target(2),q.target(3),100,'p','filled');
    for i=1:size(initial,2)
        text(ax,initial(1,i),initial(2,i),initial(3,i),sprintf(' U%d',i));
    end
    xlim(ax,q.workspace(:,1)'); ylim(ax,q.workspace(:,2)'); zlim(ax,q.workspace(:,3)');
    view(ax,38,25); daspect(ax,[1 1 0.8]);
    xlabel(ax,'x / m'); ylabel(ax,'y / m'); zlabel(ax,'z / m');
    title(ax,'Saved obstacle map and initial formation');
    ax=nexttile(layout); hold(ax,'on'); grid(ax,'on');
    gateCount=0;
    if strcmp(scene.family,'window'), gateCount=1; end
    if strcmp(scene.family,'staggered_windows'), gateCount=2; end
    for gate=1:gateCount
        lower=scene.obstacles(2*gate-1).zmax;
        upper=scene.obstacles(2*gate).zmin;
        lo=lower+q.pathClearance; hi=upper-q.pathClearance;
        fill(ax,gate+[-0.3 0.3 0.3 -0.3],[0 0 lower lower],[0.57 0.60 0.65], ...
            'EdgeColor','none','FaceAlpha',0.7);
        fill(ax,gate+[-0.3 0.3 0.3 -0.3],[upper upper 30 30],[0.40 0.49 0.62], ...
            'EdgeColor','none','FaceAlpha',0.7);
        fill(ax,gate+[-0.3 0.3 0.3 -0.3],[lo lo hi hi],[0.13 0.65 0.45], ...
            'EdgeColor','none','FaceAlpha',0.6);
        text(ax,gate,(lo+hi)/2,sprintf(' %.2f - %.2f m',lo,hi), ...
            'VerticalAlignment','middle','FontSize',10,'BackgroundColor','w');
    end
    for height=q.heightLayers
        yline(ax,height,'--',sprintf('fixed layer %g m',height), ...
            'Color',[0.78 0.23 0.18],'LabelHorizontalAlignment','left');
    end
    xlim(ax,[0.5 max(1,gateCount)+0.7]); ylim(ax,[0 30]);
    xticks(ax,1:gateCount); xticklabels(ax,compose('gate %d',1:gateCount));
    xlabel(ax,'Full-width structural window'); ylabel(ax,'height / m');
    title(ax,'Green: available height after 4.2 m planning clearance');
    sgtitle(layout,sprintf('seed %d | %s | %s | no flight history', ...
        seed,scene.family,data.row.reason),'Interpreter','none');
    exportgraphics(fig,outputFile,'Resolution',140);
    [folder,stem]=fileparts(outputFile);
    savefig(fig,fullfile(folder,[stem '.fig']));
end

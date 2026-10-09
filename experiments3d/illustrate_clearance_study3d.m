function illustrate_clearance_study3d(studyDir)
% ILLUSTRATE_CLEARANCE_STUDY3D Traceable examples selected by a declared rule.
    snap=fullfile(studyDir,'source_snapshot'); addpath(fullfile(snap,'src3d'));
    rows=readtable(fullfile(studyDir,'holdout_results.csv'),'Delimiter',',','VariableNamingRule','preserve','TextType','string');
    control=rows(rows.configId=="baseline",:); candidate=rows(rows.configId~="baseline",:);
    candidate=sortrows(candidate,'seed'); examples=struct([]);
    for family=["window","staggered_windows"]
        eligible=[];
        for k=1:height(candidate)
            c=candidate(k,:); b=control(control.seed==c.seed,:);
            if c.family==family && c.success && ~b.success, eligible=k; break; end
        end
        if isempty(eligible), continue; end
        row=candidate(eligible,:); raw=find_raw(studyDir,'holdout',row.jobId);
        x=load(raw,'r'); output=fullfile(studyDir,'examples',char(family)); mkdir(output);
        plot_result3d(x.r,output);
        examples=[examples;struct('kind',family,'seed',row.seed,'jobId',row.jobId, ...
            'sourceMat',string(raw),'reason',row.reason,'rule',"first seed in family rescued in independent validation")]; %#ok<AGROW>
    end
    % Failures are shown too; first non-planning failure under candidate noise.
    boundary=readtable(fullfile(studyDir,'boundary_results.csv'),'Delimiter',',','VariableNamingRule','preserve','TextType','string');
    eligible=boundary(boundary.configId~="baseline" & ~boundary.success & boundary.cycles>0,:);
    if ~isempty(eligible)
        eligible=sortrows(eligible,{'noiseStd','disturbanceMax','seed'}); row=eligible(1,:);
        x=load(find_raw(studyDir,'boundary',row.jobId),'r'); output=fullfile(studyDir,'examples','boundary_failure'); mkdir(output);
        plot_result3d(x.r,output);
        examples=[examples;struct('kind',"boundary_failure",'seed',row.seed,'jobId',row.jobId, ...
            'sourceMat',string(find_raw(studyDir,'boundary',row.jobId)),'reason',row.reason, ...
            'rule',"first flown candidate failure sorted by noise, disturbance, seed")];
    end
    geometry=readtable(fullfile(studyDir,'geometry_results.csv'),'Delimiter',',','VariableNamingRule','preserve','TextType','string');
    widths=[1 1.25 1.5 2 3 4 6 8 10];
    openingHeight=reshape(widths(floor((geometry.seed-900001)/5)+1),[],1);
    eligible=geometry(geometry.configId~="baseline" & geometry.configId~="floor_margin" & ...
        geometry.layerPolicy=="fixed5" & ~geometry.success & geometry.cycles>0 & openingHeight>=6,:);
    if ~isempty(eligible)
        eligible=sortrows(eligible,'seed'); row=eligible(1,:);
        raw=find_raw(studyDir,'geometry',row.jobId); x=load(raw,'r');
        output=fullfile(studyDir,'examples','aperture_stagnation'); mkdir(output);
        plot_result3d(x.r,output);
        examples=[examples;struct('kind',"aperture_stagnation",'seed',row.seed,'jobId',row.jobId, ...
            'sourceMat',string(raw),'reason',row.reason, ...
            'rule',"first flown fixed-layer candidate failure in openings at least 6 m, sorted by seed")];
    end
    if ~isempty(examples), writetable(struct2table(examples),fullfile(studyDir,'examples','example_manifest.csv')); end
end
function path=find_raw(studyDir,stage,jobId)
    list=dir(fullfile(studyDir,'stages',stage,'worker_*','trials',char(jobId+".mat")));
    assert(numel(list)==1); path=fullfile(list.folder,list.name);
end

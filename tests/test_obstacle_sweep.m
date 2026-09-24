function test_obstacle_sweep()
% Meaningful integration checks: deterministic parallelism, resume, denominators.
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'src'),fullfile(root,'config'),fullfile(root,'experiments'));
    p=default_params(); p.vmax=3; p.animate=false;
    args={'ObstacleCounts',[4 20],'BandWidths',[10 30],'MinGaps',1, ...
        'Seeds',1:2,'BaseParams',p,'MakePlots',false,'BatchSize',4};
    serial=run_obstacle_sweep(args{:},'UseParallel',false,'MaxNewTrials',4);
    partial=readtable(fullfile(serial,'trials.csv'),'TextType','string');
    assert(height(partial)==4);
    run_obstacle_sweep('ResumeDir',serial,'UseParallel',false,'MakePlots',false);
    a=readtable(fullfile(serial,'trials.csv'),'TextType','string');
    assert(height(a)==8 && numel(unique(a.jobId))==8);
    parallel=run_obstacle_sweep(args{:},'UseParallel',true,'Workers',2);
    b=readtable(fullfile(parallel,'trials.csv'),'TextType','string');
    fields=setdiff(a.Properties.VariableNames,{'wallTime_s'});
    assert(isequaln(a(:,fields),b(:,fields)), 'Parallel and serial results differ.');
    before=dir(fullfile(serial,'chunks','chunk_*.mat'));
    run_obstacle_sweep('ResumeDir',serial,'UseParallel',false,'MakePlots',false);
    after=dir(fullfile(serial,'chunks','chunk_*.mat'));
    assert(numel(before)==numel(after),'Resume reran completed jobs.');
    % Test mathematically known denominators with explicit failure categories.
    c=table(1,4,10,1,'VariableNames',{'combinationId','nObstacles','corridorBand','marginObs'});
    synthetic=repmat(a(1,:),5,1); synthetic.combinationId(:)=1;
    synthetic.outcome=["success";"success";"collision";"timeout";"generation_failed"];
    s=obstacle_sweep_summary(synthetic,c,1:5);
    assert(s.generated==4 && s.successPct==50 && s.overallSuccessPct==40 && s.generationFailurePct==20);
    assert(abs(s.ci95LowPct-15.0038989369)<1e-6 && abs(s.ci95HighPct-84.9961010631)<1e-6);
    synthetic.outcome(:)="generation_failed";
    s=obstacle_sweep_summary(synthetic,c,1:5);
    assert(isnan(s.successPct)&&s.overallSuccessPct==0&&s.generationFailurePct==100);
    synthetic.outcome(1)="execution_error";
    s=obstacle_sweep_summary(synthetic,c,1:5);
    assert(isnan(s.successPct)&&isnan(s.overallSuccessPct)&&s.executionErrors==1);
    % Actual impossible placement must become generation_failed, not collision.
    p.maxObstacleSetAttempts=1; p.maxObstaclePlacementAttempts=1;
    job=table(1,1,1,2,0,1000,'VariableNames', ...
        {'jobId','combinationId','seed','nObstacles','corridorBand','marginObs'});
    row=obstacle_sweep_trial(job,p);
    assert(row.outcome=="generation_failed"&&isnan(row.simTime_s));
    fprintf('PASS obstacle sweep: serial/parallel equality, resume, denominators, Wilson CI, generation failure.\n');
    fprintf('TEST_SERIAL_DIR=%s\nTEST_PARALLEL_DIR=%s\n',serial,parallel);
end

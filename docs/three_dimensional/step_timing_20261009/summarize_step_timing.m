function summarize_step_timing()
% Read existing frozen trials; do not rerun the controller.
base='F:\UAV project\UAV_APF_ADP_Formation';
root=fullfile(base,'results','normal_random_v2.3.3_2000_20261009');
out=fileparts(mfilename('fullpath'));
files=dir(fullfile(root,'workers','worker_*','trials','normal_*.mat'));
expected=readtable(fullfile(root,'all_flight.csv'),'TextType','string');
n=numel(files);assert(n==2000 && height(expected)==2000);
samples=cell(n,1);ordinary=cell(n,1);
seeds=zeros(n,1);initial=zeros(n,1);cycles=zeros(n,1);over=zeros(n,1);
maxMs=zeros(n,1);meanMs=zeros(n,1);terminalIndex=nan(n,1);
terminalCalls=zeros(n,1);replanCalls=zeros(n,1);
terminalSumMs=nan(n,1);terminalCycleMs=nan(n,1);
success=false(n,1);reasons=strings(n,1);relativeFiles=strings(n,1);
terminalTimes=zeros(0,1);replanTimes=zeros(0,1);slow=zeros(0,7);
elapsed=tic;
for j=1:n
    file=fullfile(files(j).folder,files(j).name);
    d=load(file,'r','q','row','errorText');r=d.r;q=d.q;assert(isempty(d.errorText));
    seed=q.seed;seeds(j)=seed;match=find(expected.seed==seed);assert(isscalar(match));
    a=r.history.cycleMs(:);
    assert(numel(a)==expected.cycles(match) && numel(a)==d.row.cycles);
    assert(~isempty(a) && all(isfinite(a)) && all(a>=0));
    assert(q.dt==0.05 && strcmp(q.version,'2.3.3'));
    assert(r.success==logical(expected.success(match)));
    assert(abs(r.time-expected.time(match))<1e-8);
    samples{j}=a;cycles(j)=numel(a);over(j)=nnz(a>50);
    maxMs(j)=max(a);meanMs(j)=mean(a);
    initial(j)=r.timing.initialPlanSeconds*1000;
    tm=r.timing.terminalPlanSeconds(:)*1000;rp=r.timing.replanSeconds(:)*1000;
    terminalTimes=[terminalTimes;tm];replanTimes=[replanTimes;rp];
    terminalCalls(j)=numel(tm);replanCalls(j)=numel(rp);
    ti=find(r.history.holdAlpha>=1,1,'first');
    if ~isempty(tm)
        assert(~isempty(ti) && numel(tm)==q.nUAV-1);
        terminalIndex(j)=ti;terminalSumMs(j)=sum(tm);terminalCycleMs(j)=a(ti);
        assert(terminalSumMs(j)<=a(ti)+1e-4);
        ordinary{j}=a([1:ti-1,ti+1:end]);
    else
        ordinary{j}=a;
    end
    idx=find(a>50);
    slow=[slow;repmat(seed,numel(idx),1),idx,r.history.t(idx),a(idx),...
        double(idx==terminalIndex(j)),repmat(expected.obstacleCount(match),numel(idx),1),...
        r.history.safetyIterations(idx)];
    success(j)=r.success;reasons(j)=string(r.reason);
    relativeFiles(j)=strrep(string(file),string(root)+filesep,"");
    if mod(j,250)==0,fprintf('TIMING_PROGRESS %d/%d elapsed=%.1fs\n',j,n,toc(elapsed));end
end
assert(numel(unique(seeds))==2000 && isequal(sort(seeds),(10001:12000)'));
assert(sum(cycles)==sum(expected.cycles));
allMs=vertcat(samples{:});otherMs=vertcat(ordinary{:});
summary=struct('version','2.3.3','sourceCommit','57d735e903ebc85485e79c8066e0f7a420eb1bf8',...
    'seeds',[10001 12000],'records',n,'deadlineMs',50,...
    'scope','Six concurrent single-compute-thread MATLAB processes; five UAVs in one cycle; initial plan excluded.',...
    'quantileMethod','P90/P95/P99/P99.9 nearest rank over all pooled steps; median uses MATLAB median.',...
    'cycles',describe(allMs),'excludingTerminalPlanningStep',describe(otherMs),...
    'terminalPlanningSteps',describe(terminalCycleMs(isfinite(terminalCycleMs))),...
    'initialPlanningMs',describe(initial),'terminalPlanningCallsMs',describe(terminalTimes),...
    'replanningCallsMs',describe(replanTimes),...
    'flightsWithAnyOver50',nnz(over>0),'flightsWithoutAnyOver50',nnz(over==0),...
    'flightOver50Percent',100*nnz(over>0)/n,...
    'terminalPlanningStepOver50',nnz(terminalCycleMs>50),'nonTerminalStepOver50',nnz(otherMs>50),...
    'flightsWithReplanning',nnz(replanCalls>0),'extractionWallSeconds',toc(elapsed));
edges=[0 5 10 20 30 40 50 100 500 1000 5000 10000 inf];
labels=["0-5","5-10","10-20","20-30","30-40","40-50","50-100",...
    "100-500","500-1000","1000-5000","5000-10000",">10000"]';
counts=zeros(numel(edges)-1,1);
for k=1:numel(counts)
    if k==1,counts(k)=nnz(allMs>=edges(k) & allMs<=edges(k+1));
    else,counts(k)=nnz(allMs>edges(k) & allMs<=edges(k+1));end
end
assert(sum(counts)==numel(allMs));
histogram=table(labels,counts,100*counts/numel(allMs),'VariableNames',{'rangeMs','steps','percent'});
summary.histogram=table2struct(histogram);
perFlight=table(seeds,cycles,over,100*over./cycles,maxMs,meanMs,...
    terminalIndex,terminalCalls,terminalSumMs,terminalCycleMs,replanCalls,initial,success,reasons,relativeFiles,...
    'VariableNames',{'seed','cycles','over50','over50Percent','maxMs','meanMs','terminalStep',...
    'terminalCalls','terminalSumMs','terminalCycleMs','replanCalls','initialPlanMs','success','reason','rawFile'});
slowSteps=array2table(slow,'VariableNames',{'seed','step','simTime','ms','terminalPlanningStep','obstacleCount','safetyIterations'});
slowSteps=sortrows(slowSteps,'ms','descend');
summary.top20SlowSteps=table2struct(slowSteps(1:min(20,height(slowSteps)),:));
[~,ii]=sort(initial,'descend');summary.top5Initial=table2struct(perFlight(ii(1:5),{'seed','initialPlanMs'}));
writetable(sortrows(perFlight,'seed'),fullfile(out,'timing_by_seed.csv'));
writetable(slowSteps,fullfile(out,'steps_above_50ms.csv'));
writetable(histogram,fullfile(out,'step_histogram.csv'));
fid=fopen(fullfile(out,'summary.json'),'w','n','UTF-8');assert(fid>=0);
fprintf(fid,'%s',jsonencode(summary,'PrettyPrint',true));fclose(fid);
save(fullfile(out,'pooled_cycle_data.mat'),'allMs','otherMs','perFlight','-v7');
fprintf('TIMING_RESULT_JSON\n%s\n',jsonencode(summary,'PrettyPrint',true));
end
function s=describe(a)
a=a(:);assert(all(isfinite(a)) && all(a>=0));
if isempty(a),s=struct('n',0);return;end
b=sort(a);n=numel(a);rank=@(f)b(max(1,ceil(f*n)));
s=struct('n',n,'min',b(1),'mean',mean(a),'median',median(a),...
    'p90',rank(.9),'p95',rank(.95),'p99',rank(.99),'p999',rank(.999),...
    'max',b(end),'over50',nnz(a>50),'over50Percent',100*nnz(a>50)/n);
end

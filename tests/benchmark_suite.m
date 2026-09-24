function benchmark_suite(projectRoot, outputFile, firstSeed, lastSeed)
% BENCHMARK_SUITE 对同一参数与 seed 范围运行批量仿真，输出 MAT 和 CSV。
% 历史配对需传入对应版本的源码目录；当前目录的参数不代表历史版本。

oldPath = path;
cleanup = onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(projectRoot, fullfile(projectRoot,'src'), fullfile(projectRoot,'config'));
seeds = firstSeed:lastSeed;
rows = repmat(struct('seed',0,'success',false,'reason','','time',NaN, ...
    'leaderDist',NaN,'formErr',NaN,'minObsClearance',NaN, ...
    'minPairClearance',NaN,'pathLength',NaN,'unsafeSteps',0, ...
    'wallSeconds',NaN),numel(seeds),1);
cases = cell(numel(seeds),1);
for k = 1:numel(seeds)
    p = default_params();
    p.seed = seeds(k);
    p.animate = false;
    t0 = tic;
    r = run_simulation(p);
    rows(k).seed = seeds(k);
    rows(k).success = r.success;
    rows(k).reason = r.reason;
    rows(k).time = r.time;
    rows(k).leaderDist = r.leaderDistFinal;
    rows(k).formErr = r.formErrFinal;
    rows(k).wallSeconds = toc(t0);
    rows(k).unsafeSteps = sum(~r.history.predictedSafe);
    [rows(k).minObsClearance,rows(k).minPairClearance] = ...
        minimum_clearances(r);
    pL = squeeze(r.history.state(1:2,p.leaderIdx,:));
    rows(k).pathLength = sum(vecnorm(diff(pL,1,2)));
    cases{k} = r;
    fprintf('BENCH seed=%d success=%d reason=%s sim=%.2f minObs=%.3f minPair=%.3f wall=%.2f\n', ...
        rows(k).seed,rows(k).success,rows(k).reason,rows(k).time, ...
        rows(k).minObsClearance,rows(k).minPairClearance,rows(k).wallSeconds);
end
summary = struct2table(rows);
save(outputFile,'summary','cases','seeds','projectRoot','-v7.3');
writetable(summary,strrep(outputFile,'.mat','.csv'));
fprintf('TOTAL success=%d/%d collision=%d timeout=%d\n', ...
    sum(summary.success),height(summary),sum(strcmp(summary.reason,'collision')), ...
    sum(strcmp(summary.reason,'timeout')));
end

function [minObs,minPair] = minimum_clearances(result)
st = result.history.state;
n = size(st,2);
steps = size(st,3);
minObs = inf;
minPair = inf;
for j = 1:numel(result.obstacles)
    obs = result.obstacles(j);
    for i = 1:n
        dx = reshape(st(1,i,:),1,steps)-obs.pos(1);
        dy = reshape(st(2,i,:),1,steps)-obs.pos(2);
        minObs = min(minObs,min(hypot(dx,dy))-obs.r-result.params.obsSafe);
    end
end
for i = 1:n-1
    for j = i+1:n
        dx = reshape(st(1,i,:)-st(1,j,:),1,steps);
        dy = reshape(st(2,i,:)-st(2,j,:),1,steps);
        minPair = min(minPair,min(hypot(dx,dy))-result.params.uavSafe);
    end
end
end

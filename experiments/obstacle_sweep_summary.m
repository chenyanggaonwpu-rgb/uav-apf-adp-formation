function summary = obstacle_sweep_summary(rows, combinations, seeds)
% OBSTACLE_SWEEP_SUMMARY Explicit denominators and per-cell Wilson intervals.
    summary = combinations;
    names = {'planned','completed','generated','successes','collisions', ...
        'timeouts','invalidStates','generationFailures','executionErrors', ...
        'successPct','ci95LowPct','ci95HighPct','overallSuccessPct', ...
        'generationFailurePct','collisionPct','timeoutPct','meanSuccessTime_s', ...
        'meanLeaderPathRatio','meanFormationRMS_m','meanWallTime_s', ...
        'boundaryClampedScenePct'};
    for k = 1:numel(names), summary.(names{k}) = nan(height(summary),1); end
    summary.planned(:) = numel(seeds);
    for i = 1:height(combinations)
        a = rows(rows.combinationId==combinations.combinationId(i),:);
        n = height(a);
        s = sum(a.outcome=="success");
        c = sum(a.outcome=="collision");
        t = sum(a.outcome=="timeout");
        v = sum(a.outcome=="invalid_state");
        f = sum(a.outcome=="generation_failed");
        e = sum(a.outcome=="execution_error");
        assert(n==s+c+t+v+f+e, 'UAV:SweepOutcome', 'Unrecognized outcome.');
        g = s+c+t+v;
        summary{i,{'completed','generated','successes','collisions','timeouts', ...
            'invalidStates','generationFailures','executionErrors'}} = [n,g,s,c,t,v,f,e];
        [low,high] = wilson(s,g);
        summary.successPct(i) = rate(s,g);
        summary.ci95LowPct(i) = 100*low;
        summary.ci95HighPct(i) = 100*high;
        % Any execution error invalidates the inferential success estimate.
        if e > 0
            summary{i,{'successPct','ci95LowPct','ci95HighPct'}} = NaN;
        end
        if e==0, summary.overallSuccessPct(i)=rate(s,n); end
        summary.generationFailurePct(i) = rate(f,n);
        summary.collisionPct(i) = rate(c,g);
        summary.timeoutPct(i) = rate(t,g);
        ok = a.outcome=="success";
        summary.meanSuccessTime_s(i) = mean(a.simTime_s(ok),'omitnan');
        summary.meanLeaderPathRatio(i) = mean(a.leaderPathRatio(ok),'omitnan');
        summary.meanFormationRMS_m(i) = mean(a.formationRMS_m(ok),'omitnan');
        summary.meanWallTime_s(i) = mean(a.wallTime_s,'omitnan');
        summary.boundaryClampedScenePct(i) = rate(sum(a.boundaryClampedCount>0),g);
    end
end

function p = rate(k,n)
    if n==0, p=NaN; else, p=100*k/n; end
end

function [lo,hi] = wilson(k,n)
    if n==0, lo=NaN; hi=NaN; return; end
    z = 1.95996398454005;
    p = k/n;
    den = 1+z*z/n;
    center = (p+z*z/(2*n))/den;
    radius = z*sqrt(p*(1-p)/n+z*z/(4*n*n))/den;
    lo=max(0,center-radius); hi=min(1,center+radius);
end

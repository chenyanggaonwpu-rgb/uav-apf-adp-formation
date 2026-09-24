function row = obstacle_sweep_trial(job, baseParams)
% OBSTACLE_SWEEP_TRIAL One deterministic trial; retain metrics, not full history.
    p = baseParams;
    p.nObstacles = job.nObstacles;
    p.corridorBand = job.corridorBand;
    p.marginObs = job.marginObs;
    p.seed = job.seed;
    p.animate = false;
    p.debug = false;
    row = table(job.jobId, job.combinationId, job.seed, p.nObstacles, ...
        p.corridorBand, p.marginObs, "", false, NaN, NaN, NaN, NaN, ...
        NaN, NaN, NaN, NaN, "", "", 'VariableNames', ...
        {'jobId','combinationId','seed','nObstacles','corridorBand','marginObs', ...
        'outcome','success','simTime_s','wallTime_s','leaderPath_m', ...
        'leaderPathRatio','formationRMS_m','formationFinal_m', ...
        'actualMinObstacleGap_m','boundaryClampedCount','errorId','errorMessage'});
    started = tic;
    try
        % Explicitly use the same generator in serial and process workers.
        rng(p.seed, 'twister');
        r = run_simulation(p);
        row.outcome = string(r.reason);
        row.success = r.success;
        row.simTime_s = r.time;
        [initial, ~] = init_formation(p);
        points = [initial(1:2,p.leaderIdx), ...
            reshape(r.history.state(1:2,p.leaderIdx,:),2,[])];
        row.leaderPath_m = sum(vecnorm(diff(points,1,2)));
        row.leaderPathRatio = row.leaderPath_m / norm(p.target-p.start);
        row.formationRMS_m = sqrt(mean(r.history.formationErr.^2));
        row.formationFinal_m = r.formErrFinal;
        row.actualMinObstacleGap_m = NaN;
        if numel(r.obstacles) > 1
            minGap = inf;
            for i = 1:numel(r.obstacles)
                for j = i+1:numel(r.obstacles)
                    minGap = min(minGap, norm(r.obstacles(i).pos-r.obstacles(j).pos) ...
                        - r.obstacles(i).r-r.obstacles(j).r);
                end
            end
            row.actualMinObstacleGap_m = minGap;
        end
        clipped = 0;
        for i = 1:numel(r.obstacles)
            o = r.obstacles(i);
            lo = p.workspace(1,:).' + o.r + 1;
            hi = p.workspace(2,:).' - o.r - 1;
            clipped = clipped + any(abs(o.pos-lo)<1e-8 | abs(o.pos-hi)<1e-8);
        end
        row.boundaryClampedCount = clipped;
    catch exception
        row.success = false;
        if strcmp(exception.identifier, 'UAV:ObstacleGenerationFailed')
            row.outcome = "generation_failed";
        else
            row.outcome = "execution_error";
        end
        row.errorId = string(exception.identifier);
        row.errorMessage = string(exception.message);
    end
    row.wallTime_s = toc(started);
end

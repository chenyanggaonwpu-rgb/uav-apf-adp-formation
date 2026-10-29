function summary = run_batch(nRuns)
% RUN_BATCH 批量运行多个随机场景并保存可复现统计结果。

    if nargin < 1
        nRuns = 20;
    end
    if ~isscalar(nRuns) || ~isfinite(nRuns) || nRuns < 1 || nRuns ~= fix(nRuns)
        error('UAV:InvalidRunCount', 'nRuns 必须是正整数。');
    end

    projectRoot = fileparts(mfilename('fullpath'));
    oldPath = path;
    pathCleanup = onCleanup(@() path(oldPath));
    addpath(fullfile(projectRoot, 'src'), fullfile(projectRoot, 'config'));
    baseParams = validate_params(default_params());
    baseParams.animate = false;
    baseParams.figDir = create_run_output_dir(baseParams.resultsRoot, ...
        'batch', baseParams.version);

    seeds = 1:nRuns;
    successFlag = false(1, nRuns);
    reasons = cell(1, nRuns);
    errorMessages = cell(1, nRuns);
    times = nan(1, nRuns);
    formErrs = nan(1, nRuns);
    leaderDistances = nan(1, nRuns);

    for s = 1:nRuns
        p = baseParams;
        p.seed = seeds(s);
        try
            r = run_simulation(p);
            successFlag(s) = r.success;
            reasons{s} = r.reason;
            times(s) = r.time;
            formErrs(s) = r.formErrFinal;
            leaderDistances(s) = r.leaderDistFinal;
            fprintf('seed %3d : %-14s time %7.2f s  form %.3f m\n', ...
                seeds(s), r.reason, r.time, r.formErrFinal);
        catch exception
            reasons{s} = 'error';
            errorMessages{s} = sprintf('%s: %s', exception.identifier, exception.message);
            fprintf('seed %3d : ERROR %s\n', seeds(s), errorMessages{s});
        end
    end

    successRate = 100 * sum(successFlag) / nRuns;
    if any(successFlag)
        meanSuccessTime = mean(times(successFlag), 'omitnan');
    else
        meanSuccessTime = nan;
    end
    meanFinalFormationError = mean(formErrs, 'omitnan');
    fprintf('\nSuccess rate: %.1f%% (%d/%d)\n', successRate, ...
        sum(successFlag), nRuns);

    summary.softwareName = baseParams.softwareName;
    summary.version = baseParams.version;
    summary.generatedAt = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
    summary.seeds = seeds;
    summary.successFlag = successFlag;
    summary.reasons = reasons;
    summary.errorMessages = errorMessages;
    summary.times = times;
    summary.formErrs = formErrs;
    summary.leaderDistances = leaderDistances;
    summary.successRate = successRate;
    summary.meanSuccessTime = meanSuccessTime;
    summary.meanFinalFormationError = meanFinalFormationError;
    summary.outputDir = baseParams.figDir;

    summaryFile = fullfile(baseParams.figDir, 'batch_summary.txt');
    fid = fopen(summaryFile, 'w', 'n', 'UTF-8');
    if fid < 0
        error('UAV:OutputFile', '无法写入批量汇总文件：%s', summaryFile);
    end
    fileCleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '%s V%s batch summary\n', baseParams.softwareName, baseParams.version);
    fprintf(fid, 'Generated: %s\n', summary.generatedAt);
    fprintf(fid, 'Runs: %d\n', nRuns);
    fprintf(fid, 'Success rate: %.1f%%\n', successRate);
    if isnan(meanSuccessTime)
        fprintf(fid, 'Mean success time: N/A (no successful runs)\n');
    else
        fprintf(fid, 'Mean success time: %.2f s\n', meanSuccessTime);
    end
    fprintf(fid, 'Mean final formation error: %.3f m\n', meanFinalFormationError);
    fprintf(fid, '\nSeed reason time formErr leaderDist error\n');
    for s = 1:nRuns
        fprintf(fid, '%d %s %.3f %.3f %.3f %s\n', seeds(s), reasons{s}, ...
            times(s), formErrs(s), leaderDistances(s), errorMessages{s});
    end
    clear fileCleanup; % 此处立即刷新并关闭文件

    save(fullfile(baseParams.figDir, 'batch_result.mat'), ...
        'summary', 'baseParams', '-v7.3');

    fig = figure('Name', 'Batch result', 'Color', 'w', 'Visible', 'off');
    figureCleanup = onCleanup(@() close(fig));
    bar(1:nRuns, double(successFlag), 0.7, 'FaceColor', [0.3 0.6 0.85]);
    ylim([0 1.1]);
    xlabel('seed');
    ylabel('success');
    title(sprintf('%s V%s — success rate %.1f%%', ...
        baseParams.softwareName, baseParams.version, successRate));
    grid on;
    print(fig, fullfile(baseParams.figDir, 'batch.png'), '-dpng', '-r150');
end

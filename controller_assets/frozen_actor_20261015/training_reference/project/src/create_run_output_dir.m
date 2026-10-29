function outputDir = create_run_output_dir(resultsRoot, prefix, version)
% CREATE_RUN_OUTPUT_DIR 为每次运行创建不会覆盖旧结果的独立目录。

    if nargin < 2 || isempty(prefix)
        prefix = 'run';
    end
    if nargin < 3 || isempty(version)
        version = 'unknown';
    end
    if ~exist(resultsRoot, 'dir')
        [ok, message] = mkdir(resultsRoot);
        if ~ok
            error('UAV:OutputDirectory', '无法创建结果根目录：%s', message);
        end
    end

    safeVersion = regexprep(version, '[^0-9A-Za-z._-]', '_');
    stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
    baseName = sprintf('%s_v%s_%s', prefix, safeVersion, stamp);
    outputDir = fullfile(resultsRoot, baseName);
    suffix = 1;
    while exist(outputDir, 'dir')
        outputDir = fullfile(resultsRoot, sprintf('%s_%02d', baseName, suffix));
        suffix = suffix + 1;
    end
    [ok, message] = mkdir(outputDir);
    if ~ok
        error('UAV:OutputDirectory', '无法创建运行结果目录：%s', message);
    end
end

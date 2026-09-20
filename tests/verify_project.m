function verify_project()
% VERIFY_PROJECT Execute tests, default entry point, rendering and 20-seed batch.
    root = fileparts(fileparts(mfilename('fullpath')));
    c = onCleanup(@() close('all'));
    oldVisibility = get(groot, 'DefaultFigureVisible');
    visibilityCleanup = onCleanup(@() set(groot, 'DefaultFigureVisible', oldVisibility));
    set(groot, 'DefaultFigureVisible', 'off');
    run_all_tests();
    p = default_params();
    p.animate = false;
    r = main(p);
    assert(r.success, 'UAV:DefaultFailed', 'Default seed failed: %s', r.reason);
    assert(isfile(fullfile(r.params.figDir, 'trajectory.png')));
    assert(isfile(fullfile(r.params.figDir, 'metrics.png')));
    assert(isfile(fullfile(r.params.figDir, 'simulation_result.mat')));
    play_flight_animation(r, r.params, 0);
    fprintf('DEFAULT_AND_RENDER_PASS output=%s\n', r.params.figDir);
    summary = run_batch(20);
    assert(~any(strcmp(summary.reasons, 'error')), 'UAV:BatchException', ...
        'Batch contained exceptions; see batch_summary.txt.');
    fprintf('BATCH_COMPLETE success=%g%% output=%s\n', summary.successRate, summary.outputDir);
    files = [dir(fullfile(root, '*.m')); dir(fullfile(root, 'src', '*.m')); ...
        dir(fullfile(root, 'config', '*.m')); dir(fullfile(root, 'tests', '*.m'))];
    for k = 1:numel(files)
        messages = checkcode(fullfile(files(k).folder, files(k).name), '-id');
        for j = 1:numel(messages)
            fprintf('CHECKCODE %s:%d %s\n', files(k).name, messages(j).line, messages(j).message);
        end
    end
    fprintf('VERIFICATION_COMPLETE\n');
end

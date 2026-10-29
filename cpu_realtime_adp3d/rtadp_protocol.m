function protocol=rtadp_protocol(project)
% CPU test settings are separate from the immutable learned/control parameters.
    protocol.version='cpu-realtime-actor-only-v3-demand';
    protocol.onlyChangeSinceV1='on-demand observation32/action3 IPC; foreground frozen supervisor uses cached Actor outputs; learned/APF/safety parameters retained';
    protocol.project=project;
    protocol.trainingRun=fullfile(project,'results','neural_adp3d', ...
        'train_20261015_222549_tp4d94db37_c59a_468e_b5c5_77ebdf5c98da');
    protocol.caseIds={'approach_window_10001','sphere_detour_40006','sequential_windows_2'};
    protocol.flightCaseIndices=[1 2 1 2 3 1];
    protocol.flightArchitectures={'serial','serial','actor_async','actor_async','actor_async','actor_async'};
    protocol.flightFaults={'none','none','none','none','none','actor_burst'};
    protocol.pipelineRepeats=100;
    protocol.inferenceRepeats=2000;
    protocol.obstacleCounts=[2 16 64 128];
    protocol.denseRepeats=30;
    protocol.actorResultLifetimeSeconds=0.2;
    protocol.actorStaleBrakeSeconds=0.4;
    protocol.paced=true;
    protocol.noTraining=true;
    protocol.noCritic=true;
    protocol.workerCount=1;
    protocol.clockBudgetSeconds=120;
    protocol.faultActorDelaySeconds=0.3;
    protocol.faultActorActiveJobs=3:5;
    protocol.delayPlantResolutionSeconds=0.05;
    protocol.delayAssumption='physical state advances for every missed 50ms control cycle; subcycle delay is not resolved';
    protocol.scope='CPU software timing and discrete delay fault tests, single UAV, known static map, zero noise';
    protocol.deadlineDefinition='finish against absolute release+50ms; wakeup lateness and local IPC count against the deadline';
    protocol.asyncBackend='two local CPU MATLAB processes; single pending job, atomic MAT files, no Parallel Toolbox pool';
end

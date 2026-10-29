function settings = apf_adp_config()
% Frozen-network supervisor. No learning, waypoint labels, or seed-dependent control.
    settings.version='apf-adp-supervisor-v1';
    settings.moduleRoot=fileparts(mfilename('fullpath'));
    settings.projectRoot=fileparts(settings.moduleRoot);
    settings.modelFile=fullfile(settings.projectRoot,'controller_assets', ...
        'frozen_actor_20261015','actor_candidate.mat');
    settings.historySeconds=3;
    settings.stallSpreadMeters=0.05;
    settings.stallProgressMeters=0.05;
    settings.stallMaxSpeed=0.08;
    settings.minimumActiveSeconds=2;
    settings.exitClearSeconds=1;
    settings.exitCorridorMeters=6;
    settings.exitExtraClearance=0.4;
    settings.exitMinimumProgress=0.5;
    settings.exitMinimumForwardSpeed=0.05;
    settings.cooldownSeconds=3;
    settings.terminalRadiusMeters=8;
    settings.maximumActiveSeconds=30;
    settings.maximumActivations=3;
    settings.maximumFlightSeconds=120;
end

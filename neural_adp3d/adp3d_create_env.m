function [env,manager] = adp3d_create_env(cfg,catalog,logDirectory)
% 构造时 RL 工具箱可能调用 reset/step 校验；这些调用不写入正式回合。
    if nargin<3, logDirectory=''; end
    manager=Adp3dEpisodeManager(cfg,catalog,logDirectory);
    before=manager.scheduleState();
    observationInfo=rlNumericSpec([32 1],'Name','observation','DataType','single');
    actionInfo=rlNumericSpec([3 1],'LowerLimit',-ones(3,1),'UpperLimit',ones(3,1),'Name','action');
    env=rlFunctionEnv(observationInfo,actionInfo, ...
        @(action,info) manager.step(action,info),@() manager.reset());
    manager.restoreSchedule(before);
    manager.LoggingEnabled=~isempty(logDirectory);
end

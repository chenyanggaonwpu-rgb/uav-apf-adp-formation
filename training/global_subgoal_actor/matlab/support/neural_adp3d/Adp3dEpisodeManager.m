classdef Adp3dEpisodeManager < handle
% 单机三维 TD3 环境：Actor 航点 -> APF -> 安全制动后备 -> 原动力学。
% 网络和优化器不在 reset 内创建；reset 只改变物理场景与飞行状态。
    properties
        Config
        Catalog
        ResetCounter=0
        ScheduleStream
        LogDirectory=''
        LoggingEnabled=false
        LastSummary=struct()
        LastInfo=struct()
        ControlMode='actor'
    end
    methods
        function obj=Adp3dEpisodeManager(cfg,catalog,logDirectory)
            obj.Config=cfg;
            obj.Catalog=catalog;
            obj.ScheduleStream=RandStream('mt19937ar','Seed',cfg.seed+17);
            if nargin>=3, obj.LogDirectory=logDirectory; end
        end

        function [observation,info]=reset(obj)
            obj.ResetCounter=obj.ResetCounter+1;
            index=randi(obj.ScheduleStream,numel(obj.Catalog));
            item=obj.Catalog(index);
            info=struct('episode',obj.ResetCounter,'caseIndex',index,'item',item, ...
                'state',item.initialState,'backup',item.initialBackup, ...
                'decisions',0,'elapsed',0,'done',false,'totalReward',0, ...
                'termination','running','states',item.initialState, ...
                'actions',zeros(3,0),'waypoints',zeros(3,0), ...
                'nominalCommands',zeros(3,0),'executedCommands',zeros(3,0), ...
                'actualAccelerations',zeros(3,0),'interventions',false(1,0), ...
                'minimumObstacleDistances',zeros(1,0),'rewards',zeros(1,0), ...
                'decisionEndIndices',zeros(1,0));
            observation=single(adp3d_observation(info.state,item.obstacles,item.parameters,obj.Config.scanRange));
            obj.LastInfo=info;
        end

        function [observation,reward,done,info]=step(obj,action,info)
            assert(~info.done,'ADP3D:EpisodeAlreadyDone');
            if iscell(action), action=action{1}; end
            action=double(action(:));
            assert(isequal(size(action),[3 1]) && all(isfinite(action)) && all(abs(action)<=1+1e-6), ...
                'ADP3D:InvalidAction');
            action=max(-1,min(1,action));
            cfg=obj.Config; p=info.item.parameters; obstacles=info.item.obstacles;
            before=norm(p.target-info.state(1:3));
            [waypoint,~]=adp3d_decode_action(info.state,action,cfg.maxOffset);
            if strcmp(obj.ControlMode,'apf'), waypoint=p.target; end
            interventions=0; collision=false; reached=false; actualSteps=0;
            for k=1:cfg.physicsStepsPerDecision
                nominal=adp3d_leader_command(info.state,waypoint,obstacles,p);
                [next,info.backup,detail]=adp3d_safe_step(info.state,nominal,info.backup,obstacles,p,cfg.safety);
                info.state=next;
                info.states(:,end+1)=next;
                info.nominalCommands(:,end+1)=detail.nominalCommand;
                info.executedCommands(:,end+1)=detail.executedCommand;
                info.actualAccelerations(:,end+1)=detail.actualAcceleration;
                info.interventions(end+1)=detail.intervention;
                info.minimumObstacleDistances(end+1)=detail.minimumObstacleDistance;
                interventions=interventions+detail.intervention;
                actualSteps=actualSteps+1;
                collision=detail.collision;
                reached=norm(p.target-next(1:3))<=p.goalTol && norm(next(4:6))<=p.vTol;
                if collision || reached, break; end
            end
            elapsed=actualSteps*p.dt;
            info.elapsed=info.elapsed+elapsed;
            info.decisions=info.decisions+1;
            after=norm(p.target-info.state(1:3));
            reward=cfg.reward.progress*(before-after)-cfg.reward.timePerSecond*elapsed ...
                -cfg.reward.intervention*interventions ...
                -cfg.reward.actionPerSecond*norm(action)^2*elapsed;
            reason='running';
            if collision
                reason='collision'; reward=reward+cfg.reward.collision;
            elseif reached
                reason='goal'; reward=reward+cfg.reward.success;
            elseif info.decisions>=cfg.maxDecisions
                reason='timeout'; reward=reward+cfg.reward.timeout;
            elseif info.elapsed>=cfg.stallSeconds
                count=round(cfg.stallSeconds/p.dt);
                recent=info.states(:,end-count:end);
                spread=max(vecnorm(recent(1:3,:)-recent(1:3,1),2,1));
                if spread<cfg.stallDisplacement && max(vecnorm(recent(4:6,:),2,1))<cfg.stallSpeed
                    reason='stall'; reward=reward+cfg.reward.stall;
                end
            end
            done=~strcmp(reason,'running');
            info.done=done; info.termination=reason;
            info.actions(:,end+1)=action;
            info.waypoints(:,end+1)=waypoint;
            info.rewards(end+1)=reward;
            info.totalReward=info.totalReward+reward;
            info.decisionEndIndices(end+1)=size(info.states,2);
            observation=single(adp3d_observation(info.state,obstacles,p,cfg.scanRange));
            obj.LastInfo=info;
            if done
                obj.LastSummary=struct('episode',info.episode,'seed',info.item.seed, ...
                    'termination',reason,'success',reached,'collision',collision, ...
                    'decisions',info.decisions,'flightSeconds',info.elapsed, ...
                    'reward',info.totalReward,'finalGoalDistance',after, ...
                    'interventions',sum(info.interventions), ...
                    'minimumObstacleDistance',min(info.minimumObstacleDistances));
                if obj.LoggingEnabled && cfg.logEpisodes
                    if ~isfolder(obj.LogDirectory), mkdir(obj.LogDirectory); end
                    episode=info; summary=obj.LastSummary; %#ok<NASGU>
                    save(fullfile(obj.LogDirectory,sprintf('episode_%06d.mat',info.episode)), ...
                        'episode','summary','-v7');
                end
            end
        end

        function value=scheduleState(obj)
            value=struct('resetCounter',obj.ResetCounter,'streamState',obj.ScheduleStream.State);
        end

        function restoreSchedule(obj,value)
            obj.ResetCounter=value.resetCounter;
            obj.ScheduleStream.State=value.streamState;
        end
    end
end

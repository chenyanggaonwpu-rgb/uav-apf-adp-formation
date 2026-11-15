function [actorNet,criticNet1,criticNet2,info] = adp3d_create_networks(cfg)
% Actor 32->64->64->3；两个独立 Critic (32,3)->64->64->1。
% 不在此函数中重置随机数，更不在每个飞行任务中重置网络。
    h=cfg.hiddenUnits;
    actorNet=dlnetwork([
        featureInputLayer(32,'Normalization','none','Name','observation')
        fullyConnectedLayer(h(1),'Name','actor_fc1')
        reluLayer('Name','actor_relu1')
        fullyConnectedLayer(h(2),'Name','actor_fc2')
        reluLayer('Name','actor_relu2')
        fullyConnectedLayer(3,'Name','actor_output')
        tanhLayer('Name','boundedAction')]);
    criticNet1=makeCritic(h);
    criticNet2=makeCritic(h);
    info=struct('actorParameters',count(actorNet), ...
        'critic1Parameters',count(criticNet1),'critic2Parameters',count(criticNet2));
    info.actorWeightBytesSingle=4*info.actorParameters;
end

function net=makeCritic(h)
    net=dlnetwork();
    net=addLayers(net,featureInputLayer(32,'Normalization','none','Name','observation'));
    net=addLayers(net,featureInputLayer(3,'Normalization','none','Name','action'));
    net=addLayers(net,[concatenationLayer(1,2,'Name','stateAction')
        fullyConnectedLayer(h(1),'Name','critic_fc1')
        reluLayer('Name','critic_relu1')
        fullyConnectedLayer(h(2),'Name','critic_fc2')
        reluLayer('Name','critic_relu2')
        fullyConnectedLayer(1,'Name','QValue')]);
    net=connectLayers(net,'observation','stateAction/in1');
    net=connectLayers(net,'action','stateAction/in2');
    net=initialize(net);
end

function n=count(net)
    n=sum(cellfun(@numel,net.Learnables.Value));
end

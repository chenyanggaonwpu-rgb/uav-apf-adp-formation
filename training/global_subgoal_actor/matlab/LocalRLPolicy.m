classdef LocalRLPolicy < handle
    properties
        Net
        Stream
        Stochastic=false
        PolicySeed=1
        NoiseHoldSeconds=.2
        RayNoiseStd=0
        RayDropout=0
        Observations=zeros(128,0,'single')
        ObservationTimes=zeros(1,0)
        Positions=zeros(3,0)
        GoalDistances=zeros(1,0)
        LastControl=zeros(3,1)
        Context=struct()
    end
    properties (Constant)
        TeacherQueries=0
        TeacherFailures=0
    end
    methods
        function obj=LocalRLPolicy(net,options,cfg) %#ok<INUSD>
            assert(~options.training&&options.teacher_probability==0,'RL:TeacherForbidden');
            obj.Net=net;obj.Stream=RandStream('mt19937ar','Seed',options.seed);
            obj.PolicySeed=options.seed;
            if isfield(options,'noise_hold_seconds'),obj.NoiseHoldSeconds=options.noise_hold_seconds;end
            obj.Stochastic=options.stochastic;obj.RayNoiseStd=options.ray_noise_std;obj.RayDropout=options.ray_dropout_probability;
        end
        function setContext(obj,measured,waypoint,time,obstacles,p,controller,lastCommand)
            obj.Context=struct('state',measured,'waypoint',waypoint,'time',time,'obstacles',obstacles,'parameters',p,'controller',controller);
            obj.LastControl=lastCommand(:,1);
            if ~isempty(obj.ObservationTimes)&&time-obj.ObservationTimes(end)<.2-1e-8,return;end
            position=measured(1:3,1);origin=max(p.workspace(1,:)',min(p.workspace(2,:)',position));
            scan=v4_scan98(origin,obstacles,p.workspace,24);
            if obj.RayNoiseStd>0,scan=max(0,min(1,scan+single(obj.RayNoiseStd/24)*randn(obj.Stream,98,1,'single')));end
            if obj.RayDropout>0,scan(rand(obj.Stream,98,1)<obj.RayDropout)=1;end
            delta=p.target-position;distance=norm(delta);goal=delta/max(distance,eps)*log1p(distance)/log1p(1000);
            wp=waypoint-position;wp=wp/max(24,norm(wp));rel=measured(1:3,2:5)-position;
            first=find(obj.ObservationTimes>=time-3,1);speed=0;spread=0;
            if ~isempty(first)
                elapsed=time-obj.ObservationTimes(first);
                if elapsed>0,speed=(obj.GoalDistances(first)-distance)/(elapsed*p.vmax);end
                spread=max(vecnorm(obj.Positions(:,first:end)-position))/(3*p.vmax);
            end
            active=max(0,time-controller.modeSince);if controller.mode~=2,active=0;end
            extra=[max(-1,min(1,speed));min(1,spread);double(min(scan));min(1,min(vecnorm(rel))/12);min(1,time/p.Tmax);min(1,active/90)];
            observation=single([goal;measured(4:6,1)/p.vmax;wp;obj.LastControl/p.amax;scan;rel(:)/12;extra]);
            assert(isequal(size(observation),[128 1])&&all(isfinite(observation)));
            obj.Observations(:,end+1)=observation;obj.ObservationTimes(end+1)=time;
            obj.Positions(:,end+1)=position;obj.GoalDistances(end+1)=distance;
        end
        function action=predict(obj,ignored) %#ok<INUSD>
            assert(~isempty(obj.Observations));n=size(obj.Observations,2);
            frames=max(1,n-2*(15:-1:0));input=obj.Observations(:,frames);input=input(:);
            [action,mu,logstd]=v4_forward(obj.Net,input);
            if obj.Stochastic
                % Off-policy exploration is coherent during critic warmup.
                % Time-keyed noise has no hidden mutable snapshot state.
                bucket=floor(obj.Context.time/max(.2,obj.NoiseHoldSeconds)+1e-8);
                noise=RandStream('mt19937ar','Seed',mod(obj.PolicySeed+7907*bucket,4294967295));
                action=tanh(mu+exp(logstd).*randn(noise,3,1,'single'));
            end
            action=double(max(-1,min(1,single(action))));
        end
    end
end

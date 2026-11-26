classdef GlobalRouteGatePolicy < RLPolicy
% Let the original G policy propose short, verified deviations from a rule route.
    properties (SetAccess=private)
        Requests=0
        LocalHopRequests=0
        UnverifiedHopRequests=0
        WindowRequests=0
        WindowHeldRequests=0
        AcceptedRequests=0
        AcceptedLocalHops=0
        AcceptedWindows=0
        RevokedRequests=0
        AppliedSteps=0
        OffRouteXYSteps=0
    end
    properties (Access=private)
        Settings
        Planner
        Controller
        RuleWaypoint=nan(3,1)
        ActivePoint=[]
        LastObservation=-inf
    end
    methods
        function obj=GlobalRouteGatePolicy(net,options,cfg)
            obj@RLPolicy(net,options,cfg);
            obj.Settings=cfg.planner;
            obj.Settings.local_hop=true;
            obj.Settings.request_mode='actor';
            obj.Controller=apf_adp_init();
            obj.Controller.proposalCalls=0;
            obj.Planner=struct('waypoint',nan(3,1),'previous',nan(3,1), ...
                'lastDecision',-inf,'bestDistance',inf,'stallSince',0,'decisions',0);
        end

        function waypoint=waypointForRoute(obj,ruleWaypoint,state,p)
            waypoint=ruleWaypoint;
            fleet=state(1:3,:);
            leader=fleet(:,1);
            if ~isequal(ruleWaypoint,obj.RuleWaypoint)
                obj.RuleWaypoint=ruleWaypoint;
                obj.ActivePoint=[];
                obj.Planner.waypoint=ruleWaypoint;
                obj.Planner.bestDistance=inf;
            end
            reserve=p.waypointTol;
            if isequal(ruleWaypoint,p.target)
                reserve=max(reserve,p.holdTriggerDist);
            end
            remaining=norm(ruleWaypoint-leader);
            if remaining<=reserve || isempty(obj.ObservationTimes)
                obj.ActivePoint=[];
                return;
            end
            observedAt=obj.ObservationTimes(end);
            if observedAt>obj.LastObservation+1e-8
                obj.LastObservation=observedAt;
                [proposal,obj.Planner,obj.Controller,diagnostic]=rl_planner_decide( ...
                    obj.Planner,obj.Controller,obj,state,observedAt,obj.Context.obstacles,p,obj.Settings);
                if diagnostic.decided
                    obj.evaluateRequest(proposal,diagnostic,fleet,ruleWaypoint,p,remaining-reserve);
                end
            end
            if isempty(obj.ActivePoint),return;end
            if norm(obj.ActivePoint-leader)<=obj.Settings.reached_meters
                obj.ActivePoint=[];
                return;
            end
            if ~obj.canJoin(fleet,obj.ActivePoint,ruleWaypoint,p)
                obj.ActivePoint=[];
                obj.RevokedRequests=obj.RevokedRequests+1;
                return;
            end
            waypoint=obj.ActivePoint;
            obj.AppliedSteps=obj.AppliedSteps+1;
            routeXY=ruleWaypoint(1:2)-leader(1:2);
            appliedXY=waypoint(1:2)-leader(1:2);
            along=max(0,dot(appliedXY,routeXY)/max(dot(routeXY,routeXY),eps));
            if norm(appliedXY-along*routeXY)>.05
                obj.OffRouteXYSteps=obj.OffRouteXYSteps+1;
            end
        end
    end
    methods (Access=private)
        function evaluateRequest(obj,proposal,diagnostic,fleet,ruleWaypoint,p,availableDistance)
            method=diagnostic.method;
            localHop=startsWith(method,'local_hop');
            unchecked=contains(method,'unverified') || ...
                startsWith(method,'no_free_start') || startsWith(method,'no_window_exit');
            window=startsWith(method,'window_bypass');
            obj.Requests=obj.Requests+1;
            obj.LocalHopRequests=obj.LocalHopRequests+localHop;
            obj.UnverifiedHopRequests=obj.UnverifiedHopRequests+unchecked;
            obj.WindowRequests=obj.WindowRequests+window;
            obj.WindowHeldRequests=obj.WindowHeldRequests+startsWith(method,'window_bypass_hold');
            obj.ActivePoint=[];
            leader=fleet(:,1);
            obj.Planner.waypoint=ruleWaypoint;
            obj.Planner.bestDistance=norm(ruleWaypoint-leader);
            if unchecked,return;end

            % Use a prefix of G's proposed segment and preserve the rule vertex.
            hopLimit=12;
            if isfield(obj.Settings,'local_hop_max') && ~isempty(obj.Settings.local_hop_max)
                hopLimit=obj.Settings.local_hop_max;
            end
            delta=proposal-leader;
            length=min([norm(delta),hopLimit,availableDistance]);
            if length<=obj.Settings.reached_meters,return;end
            candidate=leader+delta*(length/norm(delta));
            if ~obj.canJoin(fleet,candidate,ruleWaypoint,p),return;end
            obj.ActivePoint=candidate;
            obj.Planner.waypoint=candidate;
            obj.Planner.bestDistance=length;
            obj.AcceptedRequests=obj.AcceptedRequests+1;
            obj.AcceptedLocalHops=obj.AcceptedLocalHops+localHop;
            obj.AcceptedWindows=obj.AcceptedWindows+window;
        end

        function fits=canJoin(obj,fleet,candidate,ruleWaypoint,p)
            leader=fleet(:,1);
            if norm(ruleWaypoint-leader)-norm(ruleWaypoint-candidate)<.15
                fits=false;
                return;
            end
            atCandidate=fleet+(candidate-leader);
            atRule=fleet+(ruleWaypoint-leader);
            endpoints=[fleet,atCandidate,atRule];
            low=p.workspace(1,:)';high=p.workspace(2,:)';
            clearance=max(obj.Settings.projection_clearance,p.obsSafe+.05);
            fits=all(endpoints>=low & endpoints<=high,'all') && ...
                all(planner_segments_free3d([fleet,atCandidate],[atCandidate,atRule],obj.Context.obstacles,clearance));
        end
    end
end


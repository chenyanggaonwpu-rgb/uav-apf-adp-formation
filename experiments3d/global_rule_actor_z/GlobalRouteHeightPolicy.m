classdef GlobalRouteHeightPolicy < RLPolicy
% Use Actor altitude along the native leader route when both segments fit.
    properties (SetAccess=private)
        Requests=0
        AcceptedRequests=0
        RevokedRequests=0
        AppliedSteps=0
    end
    properties (Access=private)
        DecisionSeconds
        ProjectionClearance
        RuleWaypoint=nan(3,1)
        Aligning=false
        ActiveHeight=NaN
        LastRequestTime=-inf
        LastCheckTime=-inf
    end
    methods
        function obj=GlobalRouteHeightPolicy(net,options,cfg)
            obj@RLPolicy(net,options,cfg);
            obj.DecisionSeconds=cfg.planner.decision_seconds;
            obj.ProjectionClearance=cfg.planner.projection_clearance;
        end

        function point=waypointWithHeight(obj,ruleWaypoint,state,p)
            point=ruleWaypoint;
            if ~isequal(ruleWaypoint,obj.RuleWaypoint)
                sameXY=norm(ruleWaypoint(1:2)-obj.RuleWaypoint(1:2))<1e-8;
                obj.RuleWaypoint=ruleWaypoint;
                obj.Aligning=sameXY;
                obj.ActiveHeight=NaN;
                obj.LastRequestTime=-inf;
                obj.LastCheckTime=-inf;
            end
            fleet=state(1:3,:);
            horizontalDistance=norm(ruleWaypoint(1:2)-fleet(1:2,1));
            % Return to the rule altitude before advancing the route vertex.
            obj.Aligning=obj.Aligning || horizontalDistance<=p.waypointTol || isequal(ruleWaypoint,p.target);
            if obj.Aligning,return;end

            observedAt=obj.ObservationTimes(end);
            if observedAt>=obj.LastRequestTime+obj.DecisionSeconds-1e-8
                action=obj.predict([]);
                bounds=p.workspace(:,3);
                height=bounds(1)+.5*(max(-1,min(action(3),1))+1)*diff(bounds);
                obj.LastRequestTime=observedAt;
                obj.LastCheckTime=observedAt;
                obj.ActiveHeight=NaN;
                obj.Requests=obj.Requests+1;
                if abs(height-ruleWaypoint(3))>1e-8 && obj.heightFits(fleet,ruleWaypoint,height,p)
                    obj.ActiveHeight=height;
                    obj.AcceptedRequests=obj.AcceptedRequests+1;
                end
            elseif isfinite(obj.ActiveHeight) && observedAt>obj.LastCheckTime+1e-8
                obj.LastCheckTime=observedAt;
                if ~obj.heightFits(fleet,ruleWaypoint,obj.ActiveHeight,p)
                    obj.ActiveHeight=NaN;
                    obj.RevokedRequests=obj.RevokedRequests+1;
                end
            end
            if isfinite(obj.ActiveHeight)
                point(3)=obj.ActiveHeight;
                obj.AppliedSteps=obj.AppliedSteps+1;
            end
        end
    end
    methods (Access=private)
        function fits=heightFits(obj,fleet,ruleWaypoint,height,p)
            % Certify the parallel fleet segments to the proposed point and
            % back to the rule altitude, using the whole obstacle map.
            proposed=[ruleWaypoint(1:2);height];
            displaced=fleet+(proposed-fleet(:,1));
            rejoined=fleet+(ruleWaypoint-fleet(:,1));
            startPoints=[fleet,displaced];
            endPoints=[displaced,rejoined];
            points=[fleet,displaced,rejoined];
            low=p.workspace(1,:)';high=p.workspace(2,:)';
            clearance=max(obj.ProjectionClearance,p.obsSafe+.05);
            fits=all(points>=low & points<=high,'all') && ...
                all(planner_segments_free3d(startPoints,endPoints,obj.Context.obstacles,clearance));
        end
    end
end

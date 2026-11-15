function selection=adaptive_height_layers3d(p,obstacles,clearance)
% ADAPTIVE_HEIGHT_LAYERS3D Add missing section midpoints to a temporary graph.
% Probe centres plus a bounded grid over obstacle/overlap footprints.
% Selection is deterministic and uses geometry, not gate metadata or outcomes.
    if nargin<3, clearance=p.pathClearance; end
    base=reshape(p.heightLayers,1,[]); options=p.adaptiveHeight;
    probes=[p.start(1:2),p.target(1:2)]; probeArea=[0,0]; n=numel(obstacles);
    lows=zeros(2,n); highs=zeros(2,n);
    for i=1:n
        o=obstacles(i);
        if strcmp(o.type,'box'), radius=o.halfSize(1:2); else, radius=[o.r;o.r]; end
        centre=o.center(1:2);
        lows(:,i)=centre-radius; highs(:,i)=centre+radius;
        probes(:,end+1)=centre; probeArea(end+1)=prod(highs(:,i)-lows(:,i)); %#ok<AGROW>
        [extra,area]=footprint_probes(lows(:,i),highs(:,i),p,clearance,5);
        probes=[probes,extra]; probeArea=[probeArea,repmat(area,1,size(extra,2))]; %#ok<AGROW>
    end
    for i=1:n-1
        for j=i+1:n
            lower=max(lows(:,i),lows(:,j)); upper=min(highs(:,i),highs(:,j));
            if all(upper>=lower)
                [extra,area]=footprint_probes(lower,upper,p,clearance,3);
                probes=[probes,extra]; probeArea=[probeArea,repmat(area,1,size(extra,2))]; %#ok<AGROW>
            end
        end
    end
    [uniqueProbes,~,groups]=unique(probes','rows');
    probeArea=accumarray(groups,probeArea(:),[],@max)'; probes=uniqueProbes';
    sections=zeros(0,4); sectionArea=zeros(0,1);
    for i=1:size(probes,2)
        bands=available_height_intervals3d(probes(:,i),obstacles,p,clearance);
        sections=[sections;repmat(probes(:,i)',size(bands,1),1),bands]; %#ok<AGROW>
        sectionArea=[sectionArea;repmat(probeArea(i),size(bands,1),1)]; %#ok<AGROW>
    end
    selection=struct('enable',true,'attempted',true,'baseLayers',base, ...
        'layers',base,'addedHeights',zeros(1,0),'probeXY',probes,'probeFootprintArea',probeArea,'sectionFootprintArea',sectionArea, ...
        'policyVersion','midpoint_footprint_v2', ...
        'sections',sections,'decisions',strings(size(sections,1),1), ...
        'missingIntervals',0,'budgetRejected',0,'maxAddedLayers',options.maxAddedLayers, ...
        'intervalTolerance',options.intervalTolerance,'fixedStatus','not_run');
    tol=options.intervalTolerance;
    for i=1:size(sections,1)
        band=sections(i,3:4);
        if any(base>=band(1)-tol & base<=band(2)+tol)
            selection.decisions(i)="covered_by_base";
        else
            selection.decisions(i)="missing";
        end
    end
    missing=find(selection.decisions=="missing"); selection.missingIntervals=numel(missing);
    % Broad horizontal barriers first, then wider vertical sections and less travel.
    mids=mean(sections(missing,3:4),2); widths=diff(sections(missing,3:4),1,2);
    [~,order]=sortrows([-sectionArea(missing),-widths,abs(mids-p.start(3)),sections(missing,:)],[1 2 3 4 5 6 7]);
    for index=reshape(missing(order),1,[])
        band=sections(index,3:4);
        if any(selection.layers>=band(1)-tol & selection.layers<=band(2)+tol)
            selection.decisions(index)="covered_by_temporary";
        elseif numel(selection.addedHeights)>=options.maxAddedLayers
            selection.decisions(index)="layer_budget_exhausted";
            selection.budgetRejected=selection.budgetRejected+1;
        else
            z=mean(band);
            selection.addedHeights(end+1)=z;
            selection.layers=sort([selection.layers,z]);
            selection.decisions(index)="added_midpoint";
        end
    end
end

function [probes,area]=footprint_probes(lower,upper,p,clearance,count)
% Keep section samples inside valid workspace/grid bounds; no scene oracle.
    lo=p.workspace(1,1:2)'; hi=p.workspace(2,1:2)'; step=p.pathGridResolution;
    lower=max(lower,lo+clearance); upper=min(upper,hi-clearance);
    lower=lo+ceil((lower-lo)/step)*step; upper=lo+floor((upper-lo)/step)*step;
    probes=zeros(2,0); area=0;
    if any(upper<lower), return; end
    area=prod(max(upper-lower,step));
    [xx,yy]=ndgrid(linspace(lower(1),upper(1),count),linspace(lower(2),upper(2),count));
    probes=[xx(:)';yy(:)'];
end

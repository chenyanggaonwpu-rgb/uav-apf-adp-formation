function free=directional_segments_free3d(a,b,obstacles,margins)
% DIRECTIONAL_SEGMENTS_FREE3D Batched convex supporting-plane certificates.
% Any unit normal gives a valid support plane; certification does not rely
% on the accuracy of a nearest-point normal. Convex distance derivatives
% locate useful normals. Unresolved cases retain the scalar swept check.
    margins=margins(:); as=a./margins; bs=b./margins; delta=bs-as;
    free=true(1,size(a,2));
    for o=obstacles
        pending=find(free); if isempty(pending), break; end
        if strcmp(o.type,'cylinder')
            center=[o.center(:);(o.zmin+o.zmax)/2]./margins;
        else
            center=o.center(:)./margins;
        end
        lo=zeros(1,numel(pending)); hi=ones(size(lo));
        t=max(0,min(1,sum((center-as(:,pending)).*delta(:,pending),1)./ ...
            max(sum(delta(:,pending).^2,1),eps)));
        for iteration=1:22
            if isempty(pending), break; end
            sample=a(:,pending)+(b(:,pending)-a(:,pending)).*t;
            [distance,normal]=directional_clearance3d(sample,o,margins);
            bad=distance<1-1e-9;
            if strcmp(o.type,'sphere')
                support=sqrt(sum((normal.*(o.r./margins)).^2,1));
            elseif strcmp(o.type,'box')
                support=sum(abs(normal).*(o.halfSize(:)./margins),1);
            else
                support=(o.r/margins(1))*vecnorm(normal(1:2,:))+ ...
                    (o.zmax-o.zmin)/(2*margins(3))*abs(normal(3,:));
            end
            gap=min(sum(normal.*(as(:,pending)-center),1), ...
                sum(normal.*(bs(:,pending)-center),1))-support;
            certified=gap>=1-1e-9 & ~bad;
            free(pending(bad))=false;
            keep=~bad & ~certified;
            derivative=sum(normal.*delta(:,pending),1);
            hi(derivative>0)=t(derivative>0); lo(derivative<=0)=t(derivative<=0);
            pending=pending(keep); lo=lo(keep); hi=hi(keep); t=(lo+hi)/2;
        end
        for id=pending
            free(id)=segment_directional_clearance3d(a(:,id),b(:,id),o,margins)>=1-1e-9;
        end
    end
    free=free(:);
end

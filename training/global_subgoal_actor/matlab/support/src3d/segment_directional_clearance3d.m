function clearance=segment_directional_clearance3d(a,b,obstacles,margins)
% SEGMENT_DIRECTIONAL_CLEARANCE3D Continuous, normalized swept clearance.
    if margins(1)==margins(3)
        clearance=segment_clearance3d(a,b,obstacles)/margins(1); return;
    end
    clearance=inf;
    delta=b-a; lengthScaled=norm(delta./margins(:));
    opts=optimset('TolX',1e-6,'Display','off');
    for o=obstacles
        values=directional_clearance3d([a,(a+b)/2,b],o,margins);
        lower=max([values(1)-lengthScaled,values(3)-lengthScaled, ...
            values(2)-lengthScaled/2]);
        if lower>=clearance, continue; end
        if lengthScaled<1e-12
            d=values(1);
        else
            [~,minimum]=fminbnd(@(t) directional_clearance3d(a+t*delta,o,margins),0,1,opts);
            d=min([values,minimum]);
        end
        clearance=min(clearance,d);
    end
end

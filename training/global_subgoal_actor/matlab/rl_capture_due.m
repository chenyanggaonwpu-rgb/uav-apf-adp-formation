function yes=rl_capture_due(controller,history,time,p,s,options)
    if isfield(options,'capture_active_after')
        yes=controller.mode==2&&time-controller.modeSince>=options.capture_active_after-1e-9;return;
    end
    yes=false;
    if controller.mode==2||controller.mode==5||controller.activations>=s.maximumActivations,return;end
    if controller.mode==3&&time-controller.modeSince<s.cooldownSeconds-1e-9,return;end
    if apf_adp_elapsed_due(controller,time,s),yes=true;return;end
    if isfield(s,'disableNatural')&&s.disableNatural,return;end
    n=round(s.historySeconds/p.dt)+1;
    if size(history,2)<n||time-controller.historySince<s.historySeconds-1e-9,return;end
    recent=history(:,end-n+1:end);distances=vecnorm(recent(1:3,:)-p.target);
    yes=max(vecnorm(recent(1:3,:)-recent(1:3,1)))<=s.stallSpreadMeters ...
        &&abs(distances(1)-distances(end))<=s.stallProgressMeters&&max(vecnorm(recent(4:6,:)))<=s.stallMaxSpeed ...
        &&~(distances(end)<=p.goalTol&&norm(recent(4:6,end))<=p.vTol);
end

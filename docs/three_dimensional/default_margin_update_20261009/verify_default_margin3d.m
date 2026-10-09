root='F:/UAV project/UAV_APF_ADP_Formation';
output='F:/UAV project/UAV_APF_ADP_Formation/results/default_margin_v2.3.2_20261009';
restoredefaultpath;
addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'),fullfile(root,'tests'));
p=validate_params3d(default_params3d());
assert(strcmp(p.version,'2.3.2') && p.pathClearance==1.2 && p.desiredClear==4.0);
assert(~p.directionalClearance.enable && p.directionalClearance.planningZ==1.2 && p.directionalClearance.referenceZ==4.0);
assert(p.obsSafe==0.6 && p.uavSafe==1 && p.vmax==3 && p.amax==3 && p.safety.enable && p.adaptiveHeight.enable);
result=run_simulation3d(p);
save(fullfile(output,'simulation_result.mat'),'result','-v7.3');
fprintf('DEFAULT_MARGIN_SMOKE scene=%s success=%d reason=%s time=%.2f minObs=%.6f minPair=%.6f\n', ...
    p.sceneId,result.success,result.reason,result.time,result.metrics.minObstacleClearance,result.metrics.minPairDistance);
assert(result.success && strcmp(result.reason,'success'),'The new default window demonstration did not complete.');
assert(result.metrics.minObstacleClearance>p.obsSafe && result.metrics.minPairDistance>p.uavSafe);
maxSpeed=max(vecnorm(result.history.state(4:6,:,:),2,1),[],'all');
assert(maxSpeed<=p.vmax+1e-9);
test_3d;
test_environment3d;
test_adaptive_height3d;
test_directional_clearance3d;
receipt=struct('passed',true,'version',p.version,'pathClearance',p.pathClearance, ...
    'desiredClear',p.desiredClear,'directionalEnabled',p.directionalClearance.enable, ...
    'planningZ',p.directionalClearance.planningZ,'referenceZ',p.directionalClearance.referenceZ, ...
    'scene',p.sceneId,'seed',p.seed,'success',result.success,'reason',result.reason, ...
    'simulationTime',result.time,'minObstacleClearance',result.metrics.minObstacleClearance, ...
    'minPairDistance',result.metrics.minPairDistance,'maxSpeed',maxSpeed, ...
    'tests',{{'test_3d','test_environment3d','test_adaptive_height3d','test_directional_clearance3d'}}, ...
    'scope','One current-default S8 mission and four existing regression suites; no new random-seed success-rate estimate.');
fid=fopen(fullfile(output,'validation_receipt.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(receipt,PrettyPrint=true));fclose(fid);
fprintf('DEFAULT_MARGIN_UPDATE_VALIDATION_PASSED\n');

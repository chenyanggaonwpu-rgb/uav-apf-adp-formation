from pathlib import Path
import os,sys,json,csv,hashlib,importlib.util,time,collections,math
import numpy as np
P=Path(__file__).resolve().parents[2];O=P/'results/unified_actor_20261029';E=Path(__file__).resolve().parent
os.environ['TEMP']=os.environ['TMP']=str(P/'.runtime/unified_actor_20261029/analysis_tmp')
Path(os.environ['TEMP']).mkdir(exist_ok=True);sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('mat5',E/'read_mat5.py');mat=importlib.util.module_from_spec(spec);spec.loader.exec_module(mat)
def num(v,key):
    shape,x=mat.numeric(v,key);return np.array(x,dtype=float).reshape(shape,order='F')
def scalar(v,key):return float(num(v,key).ravel()[0])
def values(v):
    p={k:scalar(v,'params.'+k) for k in ['dt','amax','vmax','obsSafe','uavSafe','holdTriggerDist']}
    return p
def dump(path,value):
    temp=path.with_suffix(path.suffix+'.part');temp.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8');temp.replace(path)
def sane(x):
    if isinstance(x,(np.floating,float)):return float(x) if np.isfinite(x) else None
    if isinstance(x,(np.integer,)):return int(x)
    if isinstance(x,(np.bool_,)):return bool(x)
    if isinstance(x,dict):return {k:sane(v) for k,v in x.items()}
    if isinstance(x,(tuple,list)):return [sane(v) for v in x]
    return x
def write_csv(path,rows):
    if not rows:return
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with path.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
def audit_one(row):
    file=O/'flight'/(row['id']+'.mat');assert hashlib.sha256(file.read_bytes()).hexdigest()==row['flightSha256']
    v=mat.variable(file,'result');p=values(v);dt=p['dt']
    if row['steps']==0:return dict(id=row['id'],passed=True,steps=0,dynamicsError=0)
    state=num(v,'history.state');state=state.reshape(6,-1,row['steps'],order='F')
    command=num(v,'history.command').reshape(3,-1,row['steps'],order='F')
    if row['campaign']=='single':
        allstate=num(v,'episode.states');initial=allstate[:,0:1];disturbance=np.zeros_like(command)
    else:
        initial=num(v,'initialFleetState');disturbance=num(v,'history.disturbance').reshape(command.shape,order='F')
    previous=np.concatenate((initial[:,:,None],state[:,:,:-1]),axis=2)
    command_lengths=np.linalg.norm(command,axis=0)
    command=command*np.minimum(1,p['amax']/np.maximum(command_lengths,np.finfo(float).eps))[None,:,:]
    actual=command+disturbance
    velocity=previous[3:]+dt*actual
    velocity*=np.minimum(1,p['vmax']/np.maximum(np.linalg.norm(velocity,axis=0),np.finfo(float).eps))[None,:,:]
    expected=np.concatenate((previous[:3]+dt*velocity,velocity),axis=0)
    error=float(np.max(np.abs(expected-state)));accelerror=float(np.max(np.abs(actual-num(v,'history.actualAcceleration').reshape(actual.shape,order='F'))))
    applied_max=float(np.linalg.norm(command,axis=0).max());requested_max=float(command_lengths.max())
    assert error<1e-10 and accelerror<1e-10 and applied_max<=p['amax']+1e-10,(row['id'],error,accelerror,applied_max,p['amax'])
    if row['campaign']!='single':assert requested_max<=p['amax']+1e-10
    assert np.linalg.norm(state[3:],axis=0).max()<=p['vmax']+1e-10
    if row['campaign']!='single':
        measured=num(v,'history.measuredState').reshape(state.shape,order='F');noise=num(v,'history.noise').reshape(command.shape,order='F')
        expected_measured=previous.copy();expected_measured[:3]+=noise
        assert np.max(np.abs(measured-expected_measured))<1e-10
    goal=num(v,'params.target').ravel();start=num(v,'params.start').ravel()
    horizontal=goal[:2]-start[:2];normal=np.array([-horizontal[1],horizontal[0]])/max(np.linalg.norm(horizontal),np.finfo(float).eps)
    if np.linalg.norm(horizontal)<1e-9:normal=np.array([0.,1.])
    sway=[]
    for vehicle in range(state.shape[1]):
        pos=normal@state[:2,vehicle,:];vel=normal@state[3:5,vehicle,:]
        strong=np.flatnonzero(np.abs(vel)>=.15)
        flips=strong[1:][np.diff(np.sign(vel[strong]))!=0]
        for k in range(3,len(flips)):
            a,b=int(flips[k-3]),int(flips[k])
            if (b-a)*dt<=5 and np.ptp(pos[a:b+1])>=.30:
                sway.append(dict(vehicle=vehicle+1,startSeconds=(a+1)*dt,endSeconds=(b+1)*dt,lateralRange=float(np.ptp(pos[a:b+1]))));break
    dz=np.diff(np.concatenate((initial[2,:,None],state[2]),axis=1),axis=1)
    extra=dict(id=row['id'],passed=True,steps=row['steps'],dynamicsError=error,accelerationError=accelerror,
        lateralSway=bool(sway),swayWindows=sway,leaderAscent=float(np.maximum(dz[0],0).sum()),leaderDescent=float(np.maximum(-dz[0],0).sum()),
        fleetAscent=float(np.maximum(dz,0).sum()),fleetDescent=float(np.maximum(-dz,0).sum()),
        leaderHeightSpan=float(np.ptp(np.r_[initial[2,0],state[2,0,:]])),
        saturatedCommandFraction=float(np.mean(command_lengths>=p['amax']-1e-8)),
        maxAccelerationG=float(np.linalg.norm(actual,axis=0).max()/9.80665))
    extra['maximumRawRequestedCommand']=requested_max;extra['maximumAppliedCommand']=applied_max
    extra['commandRecordScope']='before_plant_limit' if row['campaign']=='single' else 'after_command_limit'
    if row['campaign']=='robust':
        end_step=row['gustStartStep']-1+round(row['gustDuration']/dt);until=end_step+round(5/dt)
        extra['postPulseObservationComplete']=row['steps']>=until
    return extra
def recovery(row,rows):
    if row['campaign']!='robust' or row['gustAmplitude']==0:return None
    ref=rows.get(row['matchedReference']);out=dict(id=row['id'],reference=row['matchedReference'],recovered=False,confirmedSeconds=None,
        comparisonComplete=False,reason='missing_reference',maxPositionError=None,maxVelocityError=None)
    if ref is None:return out
    if not row['auditPassed'] or not ref['auditPassed']:out['reason']='execution_error';return out
    r=mat.variable(O/'flight'/(row['id']+'.mat'),'result');b=mat.variable(O/'flight'/(ref['id']+'.mat'),'result')
    a=num(r,'history.state').reshape(6,5,-1,order='F');base=num(b,'history.state').reshape(6,5,-1,order='F');dt=scalar(r,'params.dt')
    start=row['gustStartStep']-1+round(row['gustDuration']/dt);stop=start+round(5/dt);count=min(a.shape[2],base.shape[2])
    if count<start:out['reason']='failed_before_pulse_end';return out
    pos=np.max(np.linalg.norm(a[:3,:,:count]-base[:3,:,:count],axis=0),axis=0)
    vel=np.max(np.linalg.norm(a[3:,:,:count]-base[3:,:,:count],axis=0),axis=0)
    out['maxPositionError']=float(pos[start:min(stop,count)].max()) if count>start else None
    out['maxVelocityError']=float(vel[start:min(stop,count)].max()) if count>start else None
    out['comparisonComplete']=count>=stop;run=0
    for k in range(start,min(stop,count)):
        run=run+1 if pos[k]<=.5 and vel[k]<=.25 else 0
        if run>=round(1/dt):out.update(recovered=True,confirmedSeconds=(k+1-start)*dt,reason='recovered');return out
    out['reason']='outside_recovery_band' if count>=stop else 'incomplete_observation';return out
def summary(rows,audits):
    output=[]
    for key in sorted({(r['campaign'],r['routePolicy'],r['method'],r['safety']) for r in rows}):
        group=[r for r in rows if (r['campaign'],r['routePolicy'],r['method'],r['safety'])==key]
        success=[r for r in group if r['success']]
        output.append(dict(campaign=key[0],routePolicy=key[1],method=key[2],safety=key[3],trials=len(group),successes=len(success),
            outcomes=dict(collections.Counter(r['reason'] for r in group)),actorTrials=sum(r['actorCalls']>0 for r in group),actorCalls=sum(r['actorCalls'] for r in group),
            persistentStalls=sum(r['maxStallSeconds']>=3 for r in group),lateralSwayTrials=sum(audits.get(r['id'],{}).get('lateralSway',False) for r in group),
            obstacleViolations=sum(r['obstacleSafetyViolation'] for r in group),pairViolations=sum(r['pairSafetyViolation'] for r in group),surfaceContacts=sum(r['surfaceContact'] for r in group),
            minObstacleDistance=min((r['minObstacleDistance'] for r in group if r['minObstacleDistance'] is not None),default=None),
            minPairDistance=min((r['minPairDistance'] for r in group if r['minPairDistance'] is not None),default=None),
            successMeanTime=float(np.mean([r['flightSeconds'] for r in success])) if success else None,
            successMeanPath=float(np.mean([r['pathLength'] for r in success])) if success else None,
            successMeanFormationRms=float(np.mean([r['formationRms'] for r in success])) if success else None))
    return output
def main(watch=False):
    A=O/'independent_audits';A.mkdir(exist_ok=True);started=time.time()
    while True:
        files=sorted((O/'records').glob('*.json'));rows=[json.loads(f.read_text(encoding='utf-8')) for f in files]
        for row in rows:
            dest=A/(row['id']+'.json')
            if dest.exists():continue
            if row['reason']=='execution_error':
                if watch:continue
                raise RuntimeError('Execution error '+row['id'])
            dump(dest,sane(audit_one(row)))
        audits={p.stem:json.loads(p.read_text(encoding='utf-8')) for p in A.glob('*.json')}
        dump(O/'analysis_progress.json',dict(trials=len(rows),independentAudits=len(audits),elapsedSeconds=time.time()-started))
        print('INDEPENDENT_AUDIT',len(audits),'/3840',flush=True)
        if len(rows)==3840 and not any(r['reason']=='execution_error' for r in rows):break
        if not watch:return
        time.sleep(20)
    mapping={r['id']:r for r in rows};recoveries=[x for r in rows if (x:=recovery(r,mapping)) is not None]
    for r in rows:
        r['maxRequestedCommand']=r['maxCommand'];r['maxCommand']=audits[r['id']].get('maximumAppliedCommand',r['maxCommand'])
        r['commandRecordScope']='before_plant_limit' if r['campaign']=='single' else 'after_command_limit'
    write_csv(O/'trials.csv',rows);write_csv(O/'recovery.csv',recoveries)
    write_csv(O/'independent_audit.csv',[{k:v for k,v in a.items() if k!='swayWindows'} for a in audits.values()])
    dump(O/'summary.json',sane(summary(rows,audits)))
    retries=json.loads((O/'execution_retry_registry.json').read_text(encoding='utf-8')) if (O/'execution_retry_registry.json').exists() else {'failedAttempts':0,'retriedTasks':0}
    dump(O/'completion.json',dict(complete=True,trials=len(rows),steps=sum(r['steps'] for r in rows),
        independentAudits=len(audits),allAuditsPassed=all(a['passed'] for a in audits.values()),executionErrors=0,
        controllerIDs=sorted({r['controllerID'] for r in rows}),modelHashes=sorted({r['modelSha256'] for r in rows}),
        weightHashes=sorted({r['weightsSha256'] for r in rows}),runtimeHashes=sorted({r['runtimeSourceSha256'] for r in rows}),
        legacyADPCalls=sum(r['legacyADPCalls'] for r in rows),trainingPerformed=any(r['trainingPerformed'] for r in rows),
        recoveryTrials=len(recoveries),recovered=sum(x['recovered'] for x in recoveries),
        executionErrorAttempts=retries['failedAttempts'],retriedTasks=retries['retriedTasks'],
        finalTaskExecutionErrors=0,completedAttemptRecords=len(rows)+retries['failedAttempts'],
        workerProcessErrors=json.loads((O/'worker_resume_registry.json').read_text(encoding='utf-8')).get('processFailureEvents',0) if (O/'worker_resume_registry.json').exists() else 0,
        executionAccounting='Final task records plus archived computational-error attempts; interrupted in-flight starts are not completed attempts.'))
    print('CAMPAIGN_ANALYSIS_COMPLETE',flush=True)
if __name__=='__main__':main('--watch' in sys.argv)

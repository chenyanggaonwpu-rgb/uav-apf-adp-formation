from pathlib import Path
import json,csv,hashlib,sys,importlib.util,math
import numpy as np
P=Path(__file__).resolve().parents[2];O=P/'results/unified_actor_20261029';E=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('mat5',E/'read_mat5.py');mat=importlib.util.module_from_spec(spec);spec.loader.exec_module(mat)
def n(v,key):s,d=mat.numeric(v,key);return np.array(d).reshape(s,order='F')
def sdf(p,o):
    c=np.array(o['center'],dtype=float).reshape(-1,1,1)
    if o['type']=='sphere':return np.linalg.norm(p-c,axis=0)-o['r']
    if o['type']=='cylinder':
        radial=np.linalg.norm(p[:2]-c[:2],axis=0)-o['r'];z=np.maximum(o['zmin']-p[2],p[2]-o['zmax'])
        return np.hypot(np.maximum(radial,0),np.maximum(z,0))+np.minimum(np.maximum(radial,z),0)
    half=np.array(o['halfSize']).reshape(3,1,1);q=np.abs(p-c)-half
    return np.linalg.norm(np.maximum(q,0),axis=0)+np.minimum(np.max(q,axis=0),0)
def swept(a,b,o):
    delta=b-a
    if o['type']=='sphere':
        c=np.array(o['center']).reshape(3,1,1);t=np.clip(np.sum((c-a)*delta,axis=0)/np.maximum(np.sum(delta**2,axis=0),np.finfo(float).eps),0,1)
        return sdf(a+delta*t[None],o)
    lo=np.zeros(a.shape[1:]);hi=np.ones_like(lo)
    for _ in range(45):
        t1=lo+(hi-lo)/3;t2=hi-(hi-lo)/3;choose=sdf(a+delta*t1[None],o)<sdf(a+delta*t2[None],o)
        hi=np.where(choose,t2,hi);lo=np.where(choose,lo,t1)
    return np.minimum.reduce([sdf(a,o),sdf(b,o),sdf((a+b)/2,o),sdf(a+delta*((lo+hi)/2)[None],o)])
def main():
    rows=[json.loads(p.read_text(encoding='utf-8')) for p in sorted((O/'records').glob('*.json'))]
    assert len(rows)==3840
    cases={c['id']:c for c in json.loads((E/'dynamic_cases.json').read_text(encoding='utf-8'))}
    selected=[r for r in rows if r['campaign']=='dynamic']
    extra=['robust_M01_B_actor_S1','robust_M01_N020_n01_actor_S1','robust_M01_N050_n01_actor_S1','robust_M01_N100_n01_actor_S1',
        'robust_M01_D030_PY_actor_S1','robust_M01_D100_PY_actor_S1','robust_M01_J020_D030_PY_n01_actor_S1',
        'integration_window_63001_B_actor_S1','integration_window_63001_B_none_S1']
    selected.extend(r for r in rows if r['id'] in extra)
    folder=O/'traces';folder.mkdir(exist_ok=True);records=[];geometry=[];phenomena=[]
    headers=['time_s','vehicle','x_m','y_m','z_m','vx_mps','vy_mps','vz_mps','measured_x_m','measured_y_m','measured_z_m',
        'ux_mps2','uy_mps2','uz_mps2','ax_actual_mps2','ay_actual_mps2','az_actual_mps2',
        'noise_x_m','noise_y_m','noise_z_m','gust_x_mps2','gust_y_mps2','gust_z_mps2',
        'fleet_swept_obstacle_distance_m','fleet_swept_pair_distance_m','vehicle_endpoint_surface_distance_m',
        'formation_error_m','apf_degenerate_any','safety_triggered_any','joint_brake','actor_mode','actor_calls_cumulative']
    for row in selected:
        if row['steps']==0:continue
        v=mat.variable(O/'flight'/(row['id']+'.mat'),'result');count=row['steps'];state=n(v,'history.state').reshape(6,5,count,order='F')
        measured=n(v,'history.measuredState').reshape(6,5,count,order='F');command=n(v,'history.command').reshape(3,5,count,order='F')
        accel=n(v,'history.actualAcceleration').reshape(3,5,count,order='F');noise=n(v,'history.noise').reshape(3,5,count,order='F');gust=n(v,'history.disturbance').reshape(3,5,count,order='F')
        time=n(v,'history.t').ravel();obs=n(v,'history.obstacleClearance').ravel();pair=n(v,'history.pairDistance').ravel()
        forms=n(v,'history.formationError').ravel();deg=n(v,'history.tangentDegenerate').ravel();safe=n(v,'history.safetyTriggered').ravel();brake=n(v,'history.jointBrake').ravel()
        modes=n(v,'history.actorMode').ravel();calls=n(v,'history.actorCallCount').ravel();endpoint=np.full((5,count),np.nan)
        if row['campaign']=='dynamic':
            o=cases[row['mapID']]['obstacle'];endpoint=sdf(state[:3],o)
            initial=n(v,'initialFleetState');old=np.concatenate([initial[:,:,None],state[:,:,:-1]],axis=2)
            per=swept(old[:3],state[:3],o);recomputed=np.min(per,axis=0);err=float(np.max(np.abs(recomputed-obs)))
            assert err<2e-6,(row['id'],err)
            pair_recomputed=np.full(count,np.inf)
            for i in range(5):
                for j in range(i+1,5):
                    r=old[:3,i]-old[:3,j];dr=state[:3,i]-state[:3,j]-r
                    alpha=np.clip(-np.sum(r*dr,axis=0)/np.maximum(np.sum(dr**2,axis=0),np.finfo(float).eps),0,1)
                    pair_recomputed=np.minimum(pair_recomputed,np.linalg.norm(r+dr*alpha,axis=0))
            pair_error=float(np.max(np.abs(pair_recomputed-pair)));assert pair_error<1e-10
            geometry.append(dict(id=row['id'],obstacleMaximumError=err,pairMaximumError=pair_error,passed=True,steps=count))
        out=folder/(row['id']+'.csv')
        with out.open('w',encoding='utf-8-sig',newline='') as f:
            w=csv.writer(f);w.writerow(headers)
            for k in range(count):
                for i in range(5):
                    values=[time[k],i+1,*state[:,i,k],*measured[:3,i,k],*command[:,i,k],*accel[:,i,k],*noise[:,i,k],*gust[:,i,k],obs[k],pair[k],endpoint[i,k],forms[k],int(deg[k]),int(safe[k]),int(brake[k]),int(modes[k]),int(calls[k])]
                    w.writerow([format(x,'.10g') if isinstance(x,(float,np.floating)) and np.isfinite(x) else ('' if isinstance(x,(float,np.floating)) else x) for x in values])
        records.append(dict(id=row['id'],path=out.relative_to(O).as_posix(),rows=count*5,sha256=hashlib.sha256(out.read_bytes()).hexdigest()))
        if row['campaign']=='dynamic':
            idx=int(np.argmin(obs));pi=int(np.argmin(pair));vehicle=int(np.argmin(endpoint[:,idx]))+1
            audit=json.loads((O/'independent_audits'/(row['id']+'.json')).read_text(encoding='utf-8'))
            phenomena.append(dict(id=row['id'],success=row['success'],reason=row['reason'],actorCalls=row['actorCalls'],activations=row['activations'],
                minimumObstacle=row['minObstacleDistance'],minimumObstacleTime=float(time[idx]),closestVehicleAtMinimum=vehicle,
                minimumPair=row['minPairDistance'],minimumPairTime=float(time[pi]),persistentLowSpeedSeconds=row['maxStallSeconds'],
                repeatedLateralSway=audit.get('lateralSway',False),swayWindows=audit.get('swayWindows',[]),commandReversals=row['leaderControlReversals'],
                maxCommand=row['maxCommand'],maxActualAcceleration=row['maxActualAcceleration'],degenerateSteps=row['degenerateSteps'],
                safetySteps=row['safetySteps'],brakeSteps=row['jointBrakeSteps']))
        print('TRACE',len(records),'/',len(selected),row['id'],flush=True)
    (O/'trace_index.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
    (O/'dynamic_geometry_audit.json').write_text(json.dumps(geometry,ensure_ascii=False,indent=2),encoding='utf-8')
    (O/'dynamic_phenomena.json').write_text(json.dumps(phenomena,ensure_ascii=False,indent=2),encoding='utf-8')
    def table(rows,head):return '\n'.join(['| '+' | '.join(head)+' |','| '+' | '.join(['---']*len(head))+' |']+['| '+' | '.join(map(str,r))+' |' for r in rows])
    bad=[r for r in phenomena if not r['success'] or r['repeatedLateralSway'] or r['persistentLowSpeedSeconds']>=3]
    text='# 统一 Actor 动态检查现象与依据\n\n本表覆盖50种场景的四组运行。原始MAT保存全部状态，逐步CSV保存五机位置、速度、命令、真实总加速度、干扰、距离及触发记录。\n\n'
    text+=table([[r['id'],r['reason'],f"{r['minimumObstacle']:.6f}",f"{r['minimumObstacleTime']:.2f}",r['closestVehicleAtMinimum'],f"{r['persistentLowSpeedSeconds']:.2f}",r['actorCalls'],r['repeatedLateralSway']] for r in bad],['任务ID','结局','最小障碍净空m','发生时刻s','最近无人机','持续低速s','Actor调用','重复侧摆'])
    text+='\n\n障碍硬安全距离0.6米，机间硬安全距离1米；代码判定碰撞按这两条阈值。表中的最近无人机由该步末端距离定位，整段最小距离另经独立Python重算。\n\n'
    text+=f'独立几何复核 {len(geometry)} 条，最大障碍距离误差 {max(r["obstacleMaximumError"] for r in geometry):.3g} 米，最大同步机间距离误差 {max(r["pairMaximumError"] for r in geometry):.3g} 米。\n'
    (O/'动态现象与依据.md').write_text(text,encoding='utf-8')
    print('TRACE_EXPORT_COMPLETE',len(records),flush=True)
if __name__=='__main__':main()

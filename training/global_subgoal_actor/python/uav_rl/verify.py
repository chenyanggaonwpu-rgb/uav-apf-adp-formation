from pathlib import Path
import copy,json,os,platform,subprocess,time
import numpy as np
from scipy.io import loadmat
from .io import ROOT,atomic_json,sha256,stamp
from .queue import native_command


def native_dict(value):
    if isinstance(value,dict):return {k:native_dict(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [native_dict(v) for v in value]
    if isinstance(value,np.ndarray):return native_dict(value.tolist())
    if isinstance(value,np.generic):return value.item()
    return value


def audit_trace(path):
    d=loadmat(path,simplify_cells=True);r=d['result'];p=r['params'];h=r['history']
    if not isinstance(h,dict) or 'state' not in h:return {'passed':True,'steps':0,'physics_max_error':0.}
    state=np.asarray(h['state'],float).reshape(6,5,-1);u=np.asarray(h['command'],float).reshape(3,5,-1)
    previous=np.concatenate([np.asarray(r['initialFleetState'])[:,:,None],state[:,:,:-1]],axis=2)
    disturbance=np.asarray(h['disturbance'],float).reshape(u.shape);clipped=u*np.minimum(1.,float(p['amax'])/np.maximum(np.linalg.norm(u,axis=0,keepdims=True),np.finfo(float).eps))
    acceleration=clipped+disturbance;v=previous[3:]+float(p['dt'])*acceleration
    v*=np.minimum(1.,float(p['vmax'])/np.maximum(np.linalg.norm(v,axis=0,keepdims=True),np.finfo(float).eps))
    expected=previous.copy();expected[3:]=v;expected[:3]+=float(p['dt'])*v
    error=max(float(np.max(np.abs(expected-state))),float(np.max(np.abs(acceleration-np.asarray(h['actualAcceleration']).reshape(u.shape)))))
    assert error<1e-9,(path,error)
    assert np.max(np.linalg.norm(u,axis=0))<=float(p['amax'])+1e-9
    assert np.max(np.linalg.norm(state[3:],axis=0))<=float(p['vmax'])+1e-9
    raw=previous.copy();raw[:3]+=np.asarray(h['noise']).reshape(u.shape)
    measured=raw.copy();delay=int(p.get('v4SensorDelaySteps',0))
    if delay:
        indices=np.maximum(0,np.arange(state.shape[2])-delay);old=raw[:,:,indices]
        measured[:3]=old[:3]+(np.arange(state.shape[2])-indices)[None,None,:]*float(p['dt'])*.5*(old[3:]+raw[3:])
    if float(p['noiseStd'])>0 and float(p.get('v4FilterAlpha',1))<1:
        filtered=measured[:3,:,0].copy();alpha=float(p['v4FilterAlpha'])
        for k in range(1,state.shape[2]):
            prediction=filtered+float(p['dt'])*measured[3:,:,k];filtered=prediction+alpha*(measured[:3,:,k]-prediction);measured[:3,:,k]=filtered
    measurement=max(float(np.max(np.abs(raw-np.asarray(h['rawMeasuredState']).reshape(state.shape)))),float(np.max(np.abs(measured-np.asarray(h['measuredState']).reshape(state.shape)))))
    assert measurement<1e-9,(path,measurement)
    pair=np.full(state.shape[2],np.inf)
    for i in range(4):
        for j in range(i+1,5):
            a=previous[:3,i]-previous[:3,j];delta=(state[:3,i]-state[:3,j])-a
            t=np.clip(-(a*delta).sum(0)/np.maximum((delta*delta).sum(0),np.finfo(float).eps),0,1)
            pair=np.minimum(pair,np.linalg.norm(a+t*delta,axis=0))
    pair_error=float(np.max(np.abs(pair-np.asarray(h['pairDistance']).reshape(-1))));assert pair_error<1e-9
    from .maps import sdf
    obstacles=d['job']['scene']['obstacles']
    if isinstance(obstacles,dict):obstacles=[obstacles]
    if isinstance(obstacles,np.ndarray):obstacles=obstacles.tolist()
    a=np.moveaxis(previous[:3],0,-1).reshape(-1,3);b=np.moveaxis(state[:3],0,-1).reshape(-1,3);delta=b-a
    obstacle_min=np.full(len(a),np.inf)
    for obstacle in obstacles:
        left=np.zeros(len(a));right=np.ones(len(a))
        # Independent vectorized convex 1-D minimization, including endpoints.
        for _ in range(36):
            one=(2*left+right)/3;two=(left+2*right)/3
            lower=sdf(a+one[:,None]*delta,obstacle)<=sdf(a+two[:,None]*delta,obstacle)
            right=np.where(lower,two,right);left=np.where(lower,left,one)
        distance=np.minimum.reduce([sdf(a,obstacle),sdf(b,obstacle),sdf(a+.5*delta,obstacle),sdf(a+((left+right)/2)[:,None]*delta,obstacle)])
        obstacle_min=np.minimum(obstacle_min,distance)
    obstacle_min=obstacle_min.reshape(5,-1).min(0);native_clearance=np.asarray(h['obstacleClearance']).reshape(-1)
    obstacle_error=float(np.max(np.abs(obstacle_min-native_clearance))) if len(obstacles) else 0.
    assert obstacle_error<2e-6,(path,obstacle_error)
    bounds=np.asarray(p['workspace']);boundary=bool(np.all(state[:3]>=bounds[0,:,None,None]) and np.all(state[:3]<=bounds[1,:,None,None]))
    actual_safe=boundary and obstacle_min.min()>float(p['obsSafe']) and pair.min()>float(p['uavSafe'])
    assert actual_safe==bool(d['row']['safe'])
    # Reconstruct wind separately and verify held bounded random acceleration.
    times=np.arange(state.shape[2])*float(p['dt']);intervals=np.asarray(p['v4WindIntervals']).reshape(-1,2)
    active=np.any((times[:,None]>=intervals[:,0])&(times[:,None]<intervals[:,1]),axis=1)
    wind=np.asarray(p['v4WindBias']).reshape(3,1,1)*active[None,None,:]
    random_acceleration=disturbance-wind
    assert np.max(np.linalg.norm(random_acceleration,axis=0))<=float(p['disturbanceMax'])+1e-12
    hold=int(p['disturbanceHoldSteps']);same=np.arange(1,state.shape[2])%hold!=0
    assert np.max(np.abs(np.diff(random_acceleration,axis=2)[:,:,same]),initial=0)<1e-12
    assert int(d['row']['teacher_queries'])==0 and bool(d['audit']['passed'])
    return {'passed':True,'steps':state.shape[2],'physics_max_error':error,'measurement_max_error':measurement,
        'pair_distance_max_error':pair_error,'swept_obstacle_distance_max_error':obstacle_error,
        'safety_result_independently_reconstructed':True,'bounded_held_random_acceleration_verified':True,'wind_intervals_verified':True}


def preflight(cfg,workroot,diagnostic=False):
    import psutil,torch
    from .networks import Actor,export_actor
    workroot=Path(workroot).resolve();assert workroot.drive.upper()!='C:'
    workroot.mkdir(parents=True,exist_ok=True);folder=workroot/'preflight';folder.mkdir(exist_ok=True)
    memory=psutil.virtual_memory();disk=psutil.disk_usage(str(workroot));workers=2 if memory.available>=4*1024**3 else 1
    assert disk.free>=cfg['system']['minimum_free_disk_gib']*1024**3,'Insufficient free data-drive space.'
    if not diagnostic:assert memory.available>=1.1*1024**3,'Not enough memory for the single-worker CPU profile.'
    cuda=cfg['system']['device']=='cuda'
    if cuda:
        assert torch.cuda.is_available(),'CUDA-enabled PyTorch and a compatible NVIDIA driver are required; do not silently use CPU.'
        assert torch.cuda.get_device_properties(0).total_memory>=6*1024**3,'The provided profile requires at least6GiB dedicated CUDA memory.'
    else:assert cfg['system']['device']=='cpu'
    torch.set_num_threads(2);torch.backends.cuda.matmul.allow_tf32=False
    actor=Actor(cfg['network']).to(cfg['system']['device']);parameter_count=sum(p.numel() for p in actor.parameters());assert actor.body.layers[0].in_features==3340
    with torch.no_grad():test=actor(torch.randn(64,cfg['network']['input_size'],device=cfg['system']['device']))
    assert torch.isfinite(test).all()
    model=folder/f'export_probe_{time.time_ns()}.mat';error=export_actor(actor,model,{'test':'metadata_fields_can_be_long_without_corrupting_the_export','bc_updates':0});model.unlink()
    native=native_command('selfcheck',folder/'runtime.json','unused',workroot,180)
    receipt={'passed':True,'timestamp':stamp(),'python':platform.python_version(),'python_executable':os.sys.executable,'platform':platform.platform(),
        'cpu_threads':psutil.cpu_count(),'physical_ram_bytes':memory.total,'available_ram_bytes':memory.available,'data_drive_free_bytes':disk.free,
        'recommended_workers':min(cfg['system']['workers'],workers),'torch_version':str(torch.__version__),'torch_cuda_version':torch.version.cuda,
        'cuda_tested':cuda,'gpu_name':torch.cuda.get_device_name(0) if cuda else None,'dedicated_vram_bytes':torch.cuda.get_device_properties(0).total_memory if cuda else None,
        'actor_parameters':parameter_count,'observation_scope':'global_map','input_size':3340,'hidden':[512,512,256,256],'activation':'ReLU','export_max_error':error,'native_runtime':native,
        'native_worker_sha256':sha256(ROOT/'worker_windows/uav_rl_worker.exe'),'diagnostic_only':diagnostic}
    atomic_json(folder/'preflight.json',receipt);return receipt


def learner_check(cfg,workroot,episode=None):
    import torch
    from .sac import SAC
    from .replay import Replay,Episode,save_episode
    folder=Path(workroot)/'learner_check';folder.mkdir(parents=True,exist_ok=True);torch.set_num_threads(2)
    learner=SAC(cfg,cfg['system']['device']);rng=np.random.default_rng(777)
    if episode is None:
        obs=rng.normal(0,.2,(100,128)).astype(np.float32);ix=np.arange(20,84,dtype=np.int64)
        global_features=rng.normal(0,.2,1289).astype(np.float32);positions=rng.uniform(0,30,(100,3)).astype(np.float32)
        episode=Episode('numeric_unit_only','single_window',obs,ix,ix+1,rng.uniform(-.4,.4,(len(ix),3)).astype(np.float32),
            rng.normal(0,1,len(ix)).astype(np.float32),np.r_[np.zeros(len(ix)-1),1].astype(np.float32),np.full(len(ix),.99,np.float32),global_features,positions)
    replay=Replay(cfg);replay.add(episode)
    # Numerical optimizer tests may use generated states. Actual native RL
    # flights are a separate mandatory integration receipt, never conflated.
    def batch():
        n=cfg['learner']['batch_size'];indices=rng.integers(0,len(episode.actions),n)
        return {'states':np.asarray([np.concatenate([episode.observations[np.maximum(0,int(episode.action_index[i])-2*np.arange(15,-1,-1))].reshape(2048),episode.positions[episode.action_index[i]]/np.array([120.,120.,30.],np.float32),episode.global_features]) for i in indices]),
            'next_states':np.asarray([np.concatenate([episode.observations[np.maximum(0,int(episode.next_index[i])-2*np.arange(15,-1,-1))].reshape(2048),episode.positions[episode.next_index[i]]/np.array([120.,120.,30.],np.float32),episode.global_features]) for i in indices]),
            **{k:getattr(episode,k)[indices] for k in ['actions','rewards','done','discount']}}
    original={k:v.detach().clone() for k,v in learner.actor.state_dict().items()};critic={k:v.detach().clone() for k,v in learner.critic.state_dict().items()}
    for _ in range(3):learner.update(batch(),False)
    assert all(torch.equal(v,original[k]) for k,v in learner.actor.state_dict().items())
    assert any(not torch.equal(v,critic[k]) for k,v in learner.critic.state_dict().items())
    for _ in range(10):learner.update(batch(),True)
    assert learner.actor_updates>0 and any(not torch.equal(v,original[k]) for k,v in learner.actor.state_dict().items())
    path=folder/'checkpoint_probe.pt';torch.save(learner.state_dict(),path)
    restored=SAC(cfg,cfg['system']['device']);saved=torch.load(path,map_location='cpu',weights_only=True);restored.load_state_dict(saved)
    fixed=batch();learner.load_state_dict(copy.deepcopy(saved));a=learner.update(fixed,True)
    restored.load_state_dict(copy.deepcopy(saved));b=restored.update(fixed,True)
    maximum=max(float(torch.max(torch.abs(v-restored.actor.state_dict()[k]))) for k,v in learner.actor.state_dict().items())
    assert maximum<1e-7,(maximum,a,b)
    result={'passed':True,'actor_frozen_during_critic_only':True,'critic_updated':True,'actual_rl_actor_updates':learner.actor_updates,
        'new_bc_updates':0,'checkpoint_weights_only_loaded':True,'next_update_resume_max_error':maximum,
        'scope':'Optimizer and serialization diagnostic; independent native-flight integration is recorded separately.'}
    atomic_json(folder/'learner_check.json',result);return result

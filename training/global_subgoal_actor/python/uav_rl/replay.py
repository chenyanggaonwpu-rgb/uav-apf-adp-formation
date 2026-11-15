from collections import defaultdict,deque
from dataclasses import dataclass
from pathlib import Path
import numpy as np
from .io import digest


@dataclass
class Episode:
    episode_id:str
    group:str
    observations:np.ndarray
    action_index:np.ndarray
    next_index:np.ndarray
    actions:np.ndarray
    rewards:np.ndarray
    done:np.ndarray
    discount:np.ndarray
    global_features:np.ndarray
    positions:np.ndarray
    outcome:int=0
    map_key:str=''
    def validate(self):
        assert self.global_features.shape==(1289,) and self.positions.shape==(len(self.observations),3)
        assert np.isfinite(self.global_features).all() and np.isfinite(self.positions).all()
        n=len(self.actions);assert self.observations.ndim==2 and self.observations.shape[1]==128
        assert self.actions.shape==(n,3) and n>0
        for a in [self.observations,self.actions,self.rewards,self.discount]:assert np.isfinite(a).all()
        assert np.max(np.abs(self.actions))<=1.000001
        assert all(len(a)==n for a in [self.action_index,self.next_index,self.rewards,self.done,self.discount])
        assert np.all(self.next_index>=self.action_index) and np.max(self.next_index)<len(self.observations) and np.min(self.action_index)>=0
        assert np.isin(self.done,[0,1]).all() and np.all((self.discount>=0)&(self.discount<=1))
        assert int(self.done.sum())==1 and self.done[-1]==1,'One full-flight terminal only.'
        assert np.array_equal(self.next_index[:-1],self.action_index[1:]),'Transitions must end at the next real decision.'
    @property
    def bytes(self):return sum(getattr(self,k).nbytes for k in ['observations','action_index','next_index','actions','rewards','done','discount','global_features','positions'])
    @property
    def probe_only(self):return int(digest(self.map_key or self.episode_id)[:8],16)%10==0


def save_episode(path,ep):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(path,episode_id=ep.episode_id,group=ep.group,outcome=np.int8(ep.outcome),map_key=ep.map_key,**{k:getattr(ep,k) for k in ['observations','action_index','next_index','actions','rewards','done','discount','global_features','positions']})


def load_episode(path):
    with np.load(path,allow_pickle=False) as d:return Episode(str(d['episode_id']),str(d['group']),*[d[k].copy() for k in ['observations','action_index','next_index','actions','rewards','done','discount','global_features','positions']],outcome=int(d['outcome']) if 'outcome' in d else 0,map_key=str(d['map_key']) if 'map_key' in d else '')


class Replay:
    def __init__(self,cfg):self.cfg=cfg;self.episodes={};self.order=deque();self.count=0;self.bytes=0;self.rng=np.random.default_rng(cfg['training']['seed']+31)
    def add(self,ep):
        ep.validate()
        if ep.episode_id in self.episodes:return
        self.episodes[ep.episode_id]=ep;self.order.append(ep.episode_id);self.count+=len(ep.actions);self.bytes+=ep.bytes
        while len(self.order)>1 and (self.count>self.cfg['system']['replay_transitions'] or self.bytes>self.cfg['system']['replay_max_bytes']):
            old=self.episodes.pop(self.order.popleft());self.count-=len(old.actions);self.bytes-=old.bytes
    def eligible(self,probe=False):return [e for e in self.episodes.values() if e.probe_only==probe]
    def sample(self,n=None,probe=False):
        groups=defaultdict(list)
        for ep in self.eligible(probe):groups[ep.group].append(ep)
        assert groups,'No probe/training episodes available yet.'
        names=sorted(groups);output={k:[] for k in ['states','next_states','actions','rewards','done','discount']}
        for _ in range(n or self.cfg['learner']['batch_size']):
            group=names[int(self.rng.integers(len(names)))];episodes=groups[group];ep=episodes[int(self.rng.integers(len(episodes)))];i=int(self.rng.integers(len(ep.actions)))
            for key,ix in [('states',ep.action_index[i]),('next_states',ep.next_index[i])]:
                frames=np.maximum(0,ix-2*np.arange(15,-1,-1));output[key].append(np.concatenate([ep.observations[frames].reshape(2048),ep.positions[ix]/np.array([120.,120.,30.],np.float32),ep.global_features]))
            for key in ['actions','rewards','done','discount']:output[key].append(getattr(ep,key)[i])
        return {k:np.asarray(v,dtype=np.float32) for k,v in output.items()}
    def state_dict(self):return {'ids':list(self.order),'count':self.count,'bytes':self.bytes,'rng':self.rng.bit_generator.state}
    def load_state_dict(self,state,folder):
        self.episodes={};self.order=deque();self.count=0;self.bytes=0
        for key in state['ids']:self.add(load_episode(Path(folder)/f'{key}.npz'))
        assert self.count==state['count'] and self.bytes==state['bytes'];self.rng.bit_generator.state=state['rng']

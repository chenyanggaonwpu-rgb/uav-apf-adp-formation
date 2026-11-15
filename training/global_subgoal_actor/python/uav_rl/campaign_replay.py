"""Probability-corrected terminal strata and protected family coverage."""
from collections import Counter,defaultdict
import numpy as np
from .replay import Replay as BaseReplay
from .global_obs import state_at

class Replay(BaseReplay):
    def __init__(self,cfg):super().__init__(cfg);self.focus_families=[];self.exploration_ids=set();self.mean_policy_ids=set()
    def add(self,ep):
        ep.validate();assert ep.outcome in [1,2,3] and ep.map_key
        if ep.episode_id in self.episodes:return
        self.episodes[ep.episode_id]=ep;self.order.append(ep.episode_id);self.count+=len(ep.actions);self.bytes+=ep.bytes
        while len(self.order)>1 and (self.count>self.cfg['system']['replay_transitions'] or self.bytes>self.cfg['system']['replay_max_bytes']):
            families=Counter(e.group for e in self.episodes.values());buckets=Counter((e.group,e.outcome,e.probe_only) for e in self.episodes.values())
            candidates=[key for key in self.order if families[self.episodes[key].group]>self.cfg['campaign']['minimum_replay_episodes_per_family'] and buckets[(self.episodes[key].group,self.episodes[key].outcome,self.episodes[key].probe_only)]>1]
            maps=Counter(e.map_key for e in self.episodes.values());cap=self.cfg['campaign']['replay_diversity_soft_map_cap']
            victim=next((key for key in candidates if maps[self.episodes[key].map_key]>cap),candidates[0] if candidates else None)
            if victim is None:raise MemoryError('Replay budget cannot retain configured family/outcome floors; review capacity.')
            old=self.episodes.pop(victim);self.order.remove(victim);self.count-=len(old.actions);self.bytes-=old.bytes;self.exploration_ids.discard(victim);self.mean_policy_ids.discard(victim)

    def sample(self,n=None,probe=False):
        n=n or (64 if probe else self.cfg['learner']['batch_size']);groups=defaultdict(list)
        for ep in self.eligible(probe):groups[ep.group].append(ep)
        assert groups
        buckets={k:defaultdict(list) for k in [1,2,3]}
        for group,episodes in groups.items():
            for ep in episodes:buckets[ep.outcome][group].append(ep)
        forced=([1,2,3]*(n//4))[:3*(n//4)] if probe else [1]*(n//8)+[2]*(n//16)+[3]*(n//16)
        plan=(forced+[0]*n)[:n];fractions=Counter(code if code and buckets[code] else 0 for code in plan)
        fractions={key:value/n for key,value in fractions.items()}
        def group_weights(names):return np.asarray([1. if probe or g not in self.focus_families else 3. for g in names],float)
        def probabilities(category):
            names=sorted(category);w=group_weights(names);return names,w/w.sum()
        base_names,base_weights=probabilities(groups);base_lookup=dict(zip(base_names,base_weights))
        category_lookup={k:dict(zip(*probabilities(v))) for k,v in buckets.items() if v}
        out={k:[] for k in ['states','next_states','actions','rewards','done','discount','kind','importance']}
        for requested in plan:
            category=buckets[requested] if requested and buckets[requested] else groups
            names,weights=probabilities(category);group=names[int(self.rng.choice(len(names),p=weights))];eps=category[group]
            ep=eps[int(self.rng.integers(len(eps)))];i=len(ep.actions)-1 if requested and buckets[requested] else int(self.rng.integers(len(ep.actions)))
            base=float(base_lookup[group])/len(groups[group])/len(ep.actions)
            mixture=fractions.get(0,0)*base
            if ep.done[i]:mixture+=fractions.get(ep.outcome,0)*float(category_lookup[ep.outcome][group])/len(buckets[ep.outcome][group])
            assert mixture>0
            out['states'].append(state_at(ep,ep.action_index[i]));out['next_states'].append(state_at(ep,ep.next_index[i]))
            for key in ['actions','rewards','done','discount']:out[key].append(getattr(ep,key)[i])
            out['kind'].append(ep.outcome if ep.done[i] else 0);out['importance'].append(base/mixture)
        return {k:np.asarray(v,dtype=np.float32) for k,v in out.items()}

    def guard_panel(self,n=64):
        groups=defaultdict(list);mean_groups=defaultdict(list)
        for ep in self.eligible(False):
            if ep.outcome==1 and ep.episode_id not in self.exploration_ids:groups[ep.group].append(ep)
            if ep.outcome==1 and ep.episode_id in self.mean_policy_ids:mean_groups[ep.group].append(ep)
        for group,episodes in mean_groups.items():groups[group]=episodes
        assert groups,'Need successful self-policy training experience for preservation guard.'
        panel=[]
        for group in sorted(groups):
            ep=groups[group][int(self.rng.integers(len(groups[group])))];panel.append(state_at(ep,ep.action_index[0]))
        names=sorted(groups)
        while len(panel)<n:
            group=names[int(self.rng.integers(len(names)))];episodes=groups[group];ep=episodes[int(self.rng.integers(len(episodes)))];i=int(self.rng.integers(len(ep.actions)))
            panel.append(state_at(ep,ep.action_index[i]))
        return np.asarray(panel,np.float32)
    def state_dict(self):return {**super().state_dict(),'campaign_focus_families':self.focus_families,'campaign_exploration_ids':sorted(self.exploration_ids),'campaign_mean_policy_ids':sorted(self.mean_policy_ids)}
    def load_state_dict(self,state,folder):
        super().load_state_dict(state,folder);self.focus_families=state.get('campaign_focus_families',[]);self.exploration_ids=set(state.get('campaign_exploration_ids',[]))
        self.mean_policy_ids=set(state.get('campaign_mean_policy_ids',[]))

"""SAC only: no teacher queries, labels, cloning loss, or BC update method."""
import copy,math,random
import numpy as np
import torch
from .networks import Actor,TwinCritic,import_actor


class SAC:
    def __init__(self,cfg,device,initial=None):
        self.cfg=cfg;self.device=torch.device(device);seed=cfg['training']['seed']
        random.seed(seed);np.random.seed(seed);torch.manual_seed(seed)
        if torch.cuda.is_available():torch.cuda.manual_seed_all(seed)
        self.actor=Actor(cfg['network']).to(self.device)
        if initial is not None:import_actor(self.actor,initial)
        self.critic=TwinCritic(cfg['network']).to(self.device);self.target=copy.deepcopy(self.critic).eval()
        for p in self.target.parameters():p.requires_grad_(False)
        p=cfg['learner'];self.actor_optimizer=torch.optim.Adam(self.actor.parameters(),lr=p['actor_lr'])
        self.critic_optimizer=torch.optim.Adam(self.critic.parameters(),lr=p['critic_lr'])
        self.log_alpha=torch.nn.Parameter(torch.tensor(math.log(p['initial_alpha']),device=self.device))
        self.alpha_optimizer=torch.optim.Adam([self.log_alpha],lr=p['alpha_lr'])
        self.updates=0;self.actor_updates=0;self.rejected_actor_steps=0;self.bc_updates=0

    def tensors(self,batch):return {k:torch.as_tensor(v,dtype=torch.float32,device=self.device) for k,v in batch.items()}

    @torch.no_grad()
    def targets(self,b):
        a,logp=self.actor.sample(b['next_states']);q1,q2=self.target(b['next_states'],a)
        return b['rewards'].reshape(-1,1)+b['discount'].reshape(-1,1)*(1-b['done'].reshape(-1,1))*(torch.minimum(q1,q2)-self.log_alpha.exp()*logp)

    def policy_penalty(self):return 0.

    def update(self,batch,allow_actor=False):
        b=self.tensors(batch);target=self.targets(b);q1,q2=self.critic(b['states'],b['actions'])
        weight=b.get('importance',torch.ones_like(target)).reshape(-1,1)
        loss=(weight*((q1-target).square()+(q2-target).square())).sum()/weight.sum()
        assert torch.isfinite(loss),'Nonfinite critic loss'
        self.critic_optimizer.zero_grad(set_to_none=True);loss.backward()
        critic_grad=torch.nn.utils.clip_grad_norm_(self.critic.parameters(),self.cfg['learner']['gradient_clip'])
        assert torch.isfinite(critic_grad);self.critic_optimizer.step();self.updates+=1
        actor_loss=None;kl=None;actor_step=False
        if allow_actor and self.updates%self.cfg['learner']['actor_delay']==0:
            # Reject a single optimizer step if it moves the current policy
            # too far. This constrains RL updates, without action imitation.
            with torch.no_grad():old_mu,old_logstd=self.actor.distribution(b['states'])
            old_mu=old_mu.detach().clone();old_logstd=old_logstd.detach().clone()
            previous=copy.deepcopy(self.actor.state_dict());previous_optimizer=copy.deepcopy(self.actor_optimizer.state_dict())
            for p in self.critic.parameters():p.requires_grad_(False)
            action,logp=self.actor.sample(b['states']);aq1,aq2=self.critic(b['states'],action)
            state_weight=b.get('importance',torch.ones_like(logp)).reshape(-1,1)
            objective=(state_weight*(self.log_alpha.exp().detach()*logp-torch.minimum(aq1,aq2))).sum()/state_weight.sum()
            regularizer=self.policy_penalty();objective=objective+regularizer
            assert torch.isfinite(objective),'Nonfinite actor loss'
            self.actor_optimizer.zero_grad(set_to_none=True);objective.backward()
            actor_grad=torch.nn.utils.clip_grad_norm_(self.actor.parameters(),self.cfg['learner']['gradient_clip'])
            assert torch.isfinite(actor_grad);self.actor_optimizer.step()
            for p in self.critic.parameters():p.requires_grad_(True)
            with torch.no_grad():
                new_mu,new_logstd=self.actor.distribution(b['states'])
                divergence=(new_logstd-old_logstd+(old_logstd.mul(2).exp()+(old_mu-new_mu).square())/(2*new_logstd.mul(2).exp())-.5).sum(-1).mean()
                kl=float(divergence)
            if not math.isfinite(kl) or kl>self.cfg['learner']['maximum_actor_kl']:
                self.actor.load_state_dict(previous);self.actor_optimizer.load_state_dict(previous_optimizer)
                for group in self.actor_optimizer.param_groups:group['lr']=max(self.cfg['learner']['minimum_actor_lr'],group['lr']*.5)
                self.rejected_actor_steps+=1
            else:
                actor_step=True;self.actor_updates+=1
                alpha_loss=-(state_weight*self.log_alpha*(logp+self.cfg['learner']['target_entropy']).detach()).sum()/state_weight.sum()
                self.alpha_optimizer.zero_grad(set_to_none=True);alpha_loss.backward();self.alpha_optimizer.step()
                with torch.no_grad():self.log_alpha.clamp_(math.log(self.cfg['learner']['minimum_alpha']),math.log(self.cfg['learner']['maximum_alpha']))
            actor_loss=float(objective.detach())
        with torch.no_grad():
            for a,bp in zip(self.target.parameters(),self.critic.parameters()):a.lerp_(bp,self.cfg['learner']['tau'])
        result={'update':self.updates,'actor_updates':self.actor_updates,'bc_updates':0,'critic_loss':float(loss.detach()),
            'critic_grad_norm':float(critic_grad),'actor_loss':actor_loss,'actor_step':actor_step,'actor_kl':kl,
            'cumulative_policy_kl_penalty':float(regularizer.detach()) if actor_loss is not None and isinstance(regularizer,torch.Tensor) else 0.,
            'alpha':float(self.log_alpha.exp().detach()),'q_mean':float(torch.minimum(q1,q2).detach().mean()),
            'target_mean':float(target.mean()),'actor_lr':self.actor_optimizer.param_groups[0]['lr']}
        assert all(v is None or not isinstance(v,float) or math.isfinite(v) for v in result.values()),result
        return result

    @torch.no_grad()
    def probe(self,batch):
        b=self.tensors(batch);target=self.targets(b);q1,q2=self.critic(b['states'],b['actions'])
        td=torch.sqrt(((q1-target).square().mean()+(q2-target).square().mean())/2)
        maximum=torch.max(torch.abs(torch.cat([q1,q2,target])))
        return {'td_rmse':float(td),'maximum_q_abs':float(maximum),'finite':bool(torch.isfinite(td)&torch.isfinite(maximum))}

    def state_dict(self):
        # Only tensors and primitive containers: compatible with weights_only=True.
        numpy_state=np.random.get_state()
        return {'actor':self.actor.state_dict(),'critic':self.critic.state_dict(),'target':self.target.state_dict(),
            'actor_optimizer':self.actor_optimizer.state_dict(),'critic_optimizer':self.critic_optimizer.state_dict(),
            'alpha_optimizer':self.alpha_optimizer.state_dict(),'log_alpha':self.log_alpha.detach().cpu(),
            'updates':self.updates,'actor_updates':self.actor_updates,'rejected_actor_steps':self.rejected_actor_steps,'bc_updates':0,
            'torch_rng':torch.get_rng_state(),'cuda_rng':torch.cuda.get_rng_state_all() if torch.cuda.is_available() else [],
            'numpy_rng':{'name':numpy_state[0],'keys':numpy_state[1].tolist(),'position':int(numpy_state[2]),'gauss':int(numpy_state[3]),'cached':float(numpy_state[4])},
            'python_rng':random.getstate()}

    def load_state_dict(self,state):
        assert state['bc_updates']==0
        for name in ['actor','critic','target']:getattr(self,name).load_state_dict(state[name])
        for name in ['actor_optimizer','critic_optimizer','alpha_optimizer']:getattr(self,name).load_state_dict(state[name])
        with torch.no_grad():self.log_alpha.copy_(state['log_alpha'].to(self.device))
        for name in ['updates','actor_updates','rejected_actor_steps']:setattr(self,name,state[name])
        torch.set_rng_state(state['torch_rng'].cpu());random.setstate(state['python_rng']);n=state['numpy_rng']
        np.random.set_state((n['name'],np.asarray(n['keys'],np.uint32),n['position'],n['gauss'],n['cached']))
        if torch.cuda.is_available() and state['cuda_rng']:torch.cuda.set_rng_state_all([x.cpu() for x in state['cuda_rng']])

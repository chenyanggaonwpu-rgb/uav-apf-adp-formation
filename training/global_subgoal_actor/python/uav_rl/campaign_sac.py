"""Pure SAC with terminal-stratified probes and cumulative policy constraints."""
import copy
import numpy as np
import torch
from .sac import SAC as BaseSAC

class SAC(BaseSAC):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        if self.cfg['campaign'].get('critic_action_fusion_enabled',False):
            from .campaign_critic import ActionFusionCritic
            for twin in [self.critic,self.target]:
                twin.q1=ActionFusionCritic(twin.q1);twin.q2=ActionFusionCritic(twin.q2)
            self.target.eval()
            for p in self.target.parameters():p.requires_grad_(False)
            self.critic_optimizer=torch.optim.Adam(self.critic.parameters(),lr=self.cfg['learner']['critic_lr'])
        self.guard_actor=copy.deepcopy(self.actor).eval();self.guard_states=None;self.fixed_guard_states=None;self.cumulative_rejections=0
        self.soft_policy_beta=float(self.cfg['campaign']['soft_policy_kl_weight'])
        for p in self.guard_actor.parameters():p.requires_grad_(False)

    def set_guard_panel(self,states):
        current=torch.as_tensor(states,dtype=torch.float32,device=self.device)
        if self.fixed_guard_states is None:self.fixed_guard_states=current.detach().clone()
        self.guard_states=torch.cat([self.fixed_guard_states,current],dim=0)
    def commit_guard_anchor(self):self.guard_actor.load_state_dict(self.actor.state_dict())

    def policy_penalty(self):
        if self.guard_states is None or self.soft_policy_beta<=0:return 0.
        with torch.no_grad():old_mu,old_std=self.guard_actor.distribution(self.guard_states)
        mu,std=self.actor.distribution(self.guard_states)
        kl=(std-old_std+(old_std.mul(2).exp()+(old_mu-mu).square())/(2*std.mul(2).exp())-.5).sum(-1).mean()
        return self.soft_policy_beta*kl

    def update(self,batch,allow_actor=False):
        attempt=allow_actor and (self.updates+1)%self.cfg['learner']['actor_delay']==0
        before=None;before_actions=None
        if attempt:
            before=(copy.deepcopy(self.actor.state_dict()),copy.deepcopy(self.actor_optimizer.state_dict()),
                    self.log_alpha.detach().clone(),copy.deepcopy(self.alpha_optimizer.state_dict()),self.actor_updates)
            if self.guard_states is not None:
                with torch.no_grad():before_actions=self.actor(self.guard_states).detach().clone()
        result=super().update(batch,allow_actor)
        if result['actor_step'] and self.guard_states is not None:
            with torch.no_grad():
                s=self.guard_states;actions=self.actor(s);delta=actions-self.guard_actor(s)
                bounds=s[:,2051:2057].reshape(-1,2,3)*s.new_tensor([120.,120.,30.]);scale=(bounds[:,1]-bounds[:,0])*.5
                meters=torch.linalg.vector_norm(delta*scale,dim=1)
                step=torch.linalg.vector_norm((actions-before_actions)*scale,dim=1)
                mean=float(meters.mean());p95=float(torch.quantile(meters,.95))
                stepmean=float(step.mean());step95=float(torch.quantile(step,.95))
            result.update(cumulative_mean_shift_meters=mean,cumulative_p95_shift_meters=p95,
                physical_step_mean_shift_meters=stepmean,physical_step_p95_shift_meters=step95)
            limits=self.cfg['campaign']['policy_guard'];local=self.cfg['campaign']['policy_step_guard']
            large_step=stepmean>local['mean_meters'] or step95>local['p95_meters']
            cumulative=mean>limits['mean_meters'] or p95>limits['p95_meters']
            if large_step or cumulative:
                self.actor.load_state_dict(before[0]);self.actor_optimizer.load_state_dict(before[1])
                with torch.no_grad():self.log_alpha.copy_(before[2])
                self.alpha_optimizer.load_state_dict(before[3]);self.actor_updates=before[4];self.cumulative_rejections+=1
                for group in self.actor_optimizer.param_groups:group['lr']=max(self.cfg['learner']['minimum_actor_lr'],group['lr']*.8)
                self.soft_policy_beta=min(self.cfg['campaign']['soft_policy_beta_max'],self.soft_policy_beta*1.1)
                result.update(actor_step=False,actor_updates=self.actor_updates,cumulative_guard_rejected=True,
                    physical_step_guard_rejected=large_step,cumulative_bound_rejected=cumulative,alpha=float(self.log_alpha.exp().detach()))
            elif mean>self.cfg['campaign']['soft_policy_target_mean_meters']:
                self.soft_policy_beta=min(self.cfg['campaign']['soft_policy_beta_max'],self.soft_policy_beta*1.01)
            elif mean<self.cfg['campaign']['soft_policy_target_mean_meters']*.5:
                self.soft_policy_beta=max(self.cfg['campaign']['soft_policy_beta_min'],self.soft_policy_beta*.995)
        result['soft_policy_beta']=self.soft_policy_beta
        return result

    @torch.no_grad()
    def probe(self,batch):
        b=self.tensors(batch);target=self.targets(b);q1,q2=self.critic(b['states'],b['actions'])
        squared=((q1-target).square()+(q2-target).square()).reshape(-1)/2
        kind=b['kind'].reshape(-1).long();parts={};checks=True
        limits=self.cfg['campaign']['terminal_rmse_limits']
        for code,name in [(0,'ordinary'),(1,'success'),(2,'timeout'),(3,'unsafe')]:
            mask=kind==code;count=int(mask.sum())
            error=float(torch.sqrt(squared[mask].mean())) if count else None
            parts[name]={'samples':count,'td_rmse':error}
            if count and name!='unsafe':checks=checks and np.isfinite(error) and error<=limits[name]
            elif name in ['success','timeout']:checks=False
        maximum=torch.max(torch.abs(torch.cat([q1,q2,target])))
        weights=b['importance'].reshape(-1);error=torch.sqrt((weights*squared).sum()/weights.sum());finite=bool(torch.isfinite(error)&torch.isfinite(maximum))
        return {'td_rmse':float(error),'maximum_q_abs':float(maximum),'finite':finite,
                'terminal_checks_passed':bool(checks),'strata':parts,'unsafe_terminal_error_is_diagnostic':True,'closed_loop_safety_gate_required':True}

    def state_dict(self):
        state=super().state_dict();state['campaign_guard_actor']=self.guard_actor.state_dict();state['campaign_guard_rejections']=self.cumulative_rejections
        state['campaign_guard_reference_inputs']=None if self.fixed_guard_states is None else self.fixed_guard_states.detach().cpu();state['campaign_soft_policy_beta']=float(self.soft_policy_beta)
        return state
    def compatible_learner_state(self,state):
        if self.cfg['campaign'].get('critic_action_fusion_enabled',False):
            from .campaign_critic import compatible_state
            return compatible_state(state,self.critic)
        return state
    def load_state_dict(self,state):
        state=self.compatible_learner_state(state)
        self.soft_policy_beta=float(state.get('campaign_soft_policy_beta',self.cfg['campaign']['soft_policy_kl_weight']))
        super().load_state_dict(state)
        self.guard_actor.load_state_dict(state.get('campaign_guard_actor',state['actor']))
        self.cumulative_rejections=state.get('campaign_guard_rejections',0)
        panel=state.get('campaign_guard_reference_inputs');self.fixed_guard_states=None if panel is None else panel.to(self.device).clone()

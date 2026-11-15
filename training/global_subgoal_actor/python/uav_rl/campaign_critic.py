"""Optional zero-initialized action reinjection; the Actor is unchanged."""
import copy
import torch
from torch import nn

class ActionFusionCritic(nn.Module):
    def __init__(self,old,scale=10.):
        super().__init__();self.body=copy.deepcopy(old.body);self.output=copy.deepcopy(old.output);self.scale=float(scale)
        layer=self.body.layers[1];new=nn.Linear(layer.in_features+3,layer.out_features,device=layer.weight.device,dtype=layer.weight.dtype)
        with torch.no_grad():
            new.weight.zero_();new.weight[:,:layer.in_features].copy_(layer.weight);new.bias.copy_(layer.bias)
        self.body.layers[1]=new
    def forward(self,s,a):
        x=torch.cat([s,a],-1)
        for i,(layer,norm) in enumerate(zip(self.body.layers,self.body.norms)):
            if i==1:x=torch.cat([x,a*self.scale],-1)
            x=torch.relu(norm(layer(x)))
        return self.output(x)

def compatible_state(state,critic):
    names=['q1.body.layers.1.weight','q2.body.layers.1.weight']
    desired=critic.state_dict()
    if all(state['critic'][key].shape==desired[key].shape for key in names):return state
    out=copy.deepcopy(state)
    for section in ['critic','target']:
        for key in names:
            old=out[section][key];assert desired[key].shape==(old.shape[0],old.shape[1]+3)
            out[section][key]=torch.nn.functional.pad(old,(0,3))
    ids=[i for group in out['critic_optimizer']['param_groups'] for i in group['params']]
    parameters=list(critic.named_parameters());assert len(ids)==len(parameters)
    for pid,(name,param) in zip(ids,parameters):
        if name not in names:continue
        for key,value in out['critic_optimizer']['state'].get(pid,{}).items():
            if isinstance(value,torch.Tensor) and value.ndim==2:
                assert param.shape==(value.shape[0],value.shape[1]+3)
                out['critic_optimizer']['state'][pid][key]=torch.nn.functional.pad(value,(0,3))
    return out

from pathlib import Path
import json,os
import numpy as np
import torch
from torch import nn
from scipy.io import loadmat,savemat


class Backbone(nn.Module):
    def __init__(self,n,settings):
        super().__init__();sizes=[n]+settings['hidden']
        self.layers=nn.ModuleList(nn.Linear(a,b) for a,b in zip(sizes[:-1],sizes[1:]))
        self.norms=nn.ModuleList(nn.LayerNorm(size,eps=settings['layer_norm_epsilon']) for size in settings['hidden'])
        for layer in self.layers:nn.init.orthogonal_(layer.weight,gain=2**.5);nn.init.zeros_(layer.bias)
    def forward(self,x):
        for layer,norm in zip(self.layers,self.norms):x=torch.relu(norm(layer(x)))
        return x


class Actor(nn.Module):
    def __init__(self,settings):
        super().__init__();self.settings=settings;self.body=Backbone(settings['input_size'],settings)
        self.mu=nn.Linear(256,3);self.log_std=nn.Linear(256,3)
        nn.init.uniform_(self.mu.weight,-.003,.003);nn.init.zeros_(self.mu.bias)
        nn.init.uniform_(self.log_std.weight,-.003,.003);nn.init.constant_(self.log_std.bias,-1.)
    def distribution(self,x):
        features=self.body(x)
        return self.mu(features),self.log_std(features).clamp(self.settings['log_std_min'],self.settings['log_std_max'])
    def forward(self,x):return self.distribution(x)[0].tanh()
    def sample(self,x):
        mu,log_std=self.distribution(x);normal=torch.distributions.Normal(mu,log_std.exp());raw=normal.rsample()
        action=raw.tanh();jacobian=2*(np.log(2)-raw-torch.nn.functional.softplus(-2*raw))
        return action,(normal.log_prob(raw)-jacobian).sum(-1,keepdim=True)


class Critic(nn.Module):
    def __init__(self,settings):
        super().__init__();self.body=Backbone(settings['input_size']+3,settings);self.output=nn.Linear(256,1)
        nn.init.uniform_(self.output.weight,-.003,.003);nn.init.zeros_(self.output.bias)
    def forward(self,s,a):return self.output(self.body(torch.cat([s,a],-1)))


class TwinCritic(nn.Module):
    def __init__(self,settings):super().__init__();self.q1=Critic(settings);self.q2=Critic(settings)
    def forward(self,s,a):return self.q1(s,a),self.q2(s,a)


def import_actor(actor,path):
    data=loadmat(path,simplify_cells=True)['actor']
    assert int(data['schema'])==6 and int(data['input_size'])==actor.settings['input_size']
    with torch.no_grad():
        for i,(layer,norm) in enumerate(zip(actor.body.layers,actor.body.norms),1):
            layer.weight.copy_(torch.as_tensor(data[f'W{i}']));layer.bias.copy_(torch.as_tensor(data[f'b{i}']).reshape(-1))
            norm.weight.copy_(torch.as_tensor(data[f'g{i}']).reshape(-1));norm.bias.copy_(torch.as_tensor(data[f'n{i}']).reshape(-1))
        for prefix,head in [('mean',actor.mu),('logstd',actor.log_std)]:
            head.weight.copy_(torch.as_tensor(data[prefix+'_W']));head.bias.copy_(torch.as_tensor(data[prefix+'_b']).reshape(-1))


def numpy_forward(data,x):
    for i in range(1,5):
        x=x@data[f'W{i}'].T+np.asarray(data[f'b{i}']).reshape(1,-1)
        center=x-x.mean(-1,keepdims=True);variance=(center*center).mean(-1,keepdims=True)
        x=np.maximum(0,center/np.sqrt(variance+float(data['epsilon']))*np.asarray(data[f'g{i}']).reshape(1,-1)+np.asarray(data[f'n{i}']).reshape(1,-1))
    return np.tanh(x@data['mean_W'].T+np.asarray(data['mean_b']).reshape(1,-1))


def export_actor(actor,path,metadata):
    path=Path(path);assert not path.exists(),'Policy exports are immutable.';path.parent.mkdir(parents=True,exist_ok=True)
    cfg=actor.settings;data={'schema':np.int32(6),'action_semantics':'normalized_global_subgoal_xyz','input_size':np.int32(cfg['input_size']),'observation_size':np.int32(128),
        'global_schema':np.int32(1),'maximum_obstacles':np.int32(128),'global_feature_size':np.int32(1289),
        'coordinate_scale':np.asarray([120.,120.,30.],np.float32)[:,None],'history_frames':np.int32(16),'history_stride':np.int32(2),'epsilon':np.float32(cfg['layer_norm_epsilon']),
        'log_std_min':np.float32(cfg['log_std_min']),'log_std_max':np.float32(cfg['log_std_max'])}
    def array(t):return t.detach().cpu().numpy().astype(np.float32)
    for i,(layer,norm) in enumerate(zip(actor.body.layers,actor.body.norms),1):
        data.update({f'W{i}':array(layer.weight),f'b{i}':array(layer.bias)[:,None],f'g{i}':array(norm.weight)[:,None],f'n{i}':array(norm.bias)[:,None]})
    for prefix,head in [('mean',actor.mu),('logstd',actor.log_std)]:data[prefix+'_W']=array(head.weight);data[prefix+'_b']=array(head.bias)[:,None]
    temporary=path.with_name(path.name+'.part')
    # JSON contains arbitrary descriptive keys; MAT structure field names remain short.
    savemat(temporary,{'actor':data,'metadata_json':json.dumps(metadata,ensure_ascii=False)},appendmat=False,
        do_compression=True,long_field_names=True,oned_as='column')
    restored=loadmat(temporary,simplify_cells=True)['actor'];probe=np.random.default_rng(271828).normal(0,.3,(32,cfg['input_size'])).astype(np.float32)
    with torch.no_grad():expected=actor(torch.as_tensor(probe,device=next(actor.parameters()).device)).cpu().numpy()
    error=float(np.max(np.abs(numpy_forward(restored,probe)-expected)))
    assert error<5e-6,('MAT export parity',error)
    os.replace(temporary,path);return error

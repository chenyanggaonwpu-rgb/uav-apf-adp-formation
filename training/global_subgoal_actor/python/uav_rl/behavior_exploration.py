"""Training-only variance-boosted copies of this learner, never expert labels."""
import copy
from pathlib import Path
import torch
from .networks import export_actor
from .io import sha256

def export_explorers(actor,learned_path,settings):
    paths=[]
    for i,value in enumerate(settings['log_std_values']):
        model=copy.deepcopy(actor);model.settings=copy.deepcopy(actor.settings)
        model.settings['log_std_max']=max(float(value),.7)
        with torch.no_grad():
            model.log_std.weight.zero_();model.log_std.bias.copy_(model.log_std.bias.new_tensor([float(value),float(value),-.7]))
        path=Path(learned_path).with_name(Path(learned_path).stem+f'_explore{i}.mat')
        export_actor(model,path,{'kind':'training_only_variance_exploration','learned_parent_sha256':sha256(learned_path),
                                'exploration_log_std':float(value),'mean_parameters_unchanged':True,'teacher_queries':0,'bc_updates':0})
        paths.append(path)
    return paths

def decorate_job(trainer,job):
    settings=trainer.cfg['campaign']['behavior_exploration'];stage=trainer.state.get('campaign_stage',0)
    probability=settings['probability'] if stage<3 else settings['probability']*.5
    job['behavior_exploration']=False
    if job.get('campaign_rehearsal',False):return job
    if job['scene']['group'] in settings['hard_families'] and trainer.rng.random()<probability:
        paths=trainer.exploration_policies;path=paths[int(trainer.rng.integers(len(paths)))]
        job.update(model=str(path.resolve()),model_sha256=sha256(path),behavior_exploration=True,
                   noise_hold_seconds=settings['hold_seconds'])
    return job

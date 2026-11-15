from .io import ROOT,read_json,digest

FAMILIES=['single_window','double_window','multi_window','u_trap','c_trap','nested_u','bamboo',
          'dense_mixed','narrow_corridor','staggered_wall','slalom','maze','dead_end','fork_trap','height_chicane','mixed']
CONTEXTS=['global_planning']


def load_config(path=None):
    cfg=read_json(path or ROOT/'config/local_cpu_long.json')
    assert cfg['network']['hidden']==[512,512,256,256] and cfg['network']['activation']=='ReLU'
    assert cfg['network']['observation_size']==128 and cfg['network']['history_frames']==16 and cfg['network']['history_stride']==2
    assert cfg['network']['input_size']==3340 and cfg['network']['global_observation_schema']==1
    assert cfg['network']['maximum_obstacles']==128 and cfg['network']['global_feature_size']==1289
    assert cfg['network']['coordinate_scale']==[120.,120.,30.]
    assert cfg['training']['bc_updates']==0 and cfg['training']['teacher_probability']==0
    assert cfg['training']['actor_initialization'] in ['random','checkpoint']
    assert cfg['system']['workers'] in [1,2,3] and cfg['learner']['batch_size']<=256
    assert cfg['environment']['mission_seconds']==120 and cfg['environment']['dt']==.05
    assert cfg['dataset']['families']==FAMILIES and cfg['environment']['contexts']==CONTEXTS
    assert cfg['network']['actor_schema']==6 and cfg['network']['action_semantics']=='normalized_global_subgoal_xyz'
    assert cfg['environment']['protocol']=='global_subgoal_v1' and cfg['environment']['decision_seconds']==1.6
    assert cfg['training']['fresh_map_probability']==1. and cfg['training']['maximum_reuses_per_map']==0
    assert cfg['reward']['schema']==2 and cfg['reward']['revisit_per_second']==2.
    assert cfg['reward']['revisit_multiplier']==20. and cfg['reward']['progress_interpolation']=='trilinear'
    cfg['_sha256']=digest(cfg);return cfg

"""Audited explicit migration to a new reward run, never a same-run hash override."""
from pathlib import Path
import copy,json,os,shutil,time,ast
import numpy as np
import torch
from scipy.io import loadmat
from .io import ROOT,read_json,atomic_json,append_json,sha256,digest,source_manifest,stamp
from .config import load_config
from .campaign_sac import SAC
from .replay import save_episode,load_episode
from .campaign_replay import Replay
from .reward import read_transition
from .networks import export_actor,import_actor
from .evaluate import admissible
from .verify import native_dict
from . import checkpoint
from .reward_v2_checks import run_checks


def sources_at(root):
    return {p.relative_to(root).as_posix():sha256(p) for name in ['python','config','matlab']
        for p in sorted((root/name).rglob('*')) if p.is_file() and p.suffix in ['.py','.m','.json'] and '__pycache__' not in p.parts}


def same(a,b):
    if isinstance(a,torch.Tensor):return isinstance(b,torch.Tensor) and torch.equal(a,b)
    if isinstance(a,dict):return a.keys()==b.keys() and all(same(a[k],b[k]) for k in a)
    if isinstance(a,(list,tuple)):return len(a)==len(b) and all(same(x,y) for x,y in zip(a,b))
    return a==b


def link(source,target):
    source=Path(source).resolve();target=Path(target);target.parent.mkdir(parents=True,exist_ok=True)
    assert source.is_file() and not target.exists(),(source,target)
    try:os.link(source,target)
    except OSError:shutil.copy2(source,target)


def migrate(parent_source,parent_run,workroot,child_id):
    parent_source=Path(parent_source).resolve();parent_run=Path(parent_run).resolve();workroot=Path(workroot).resolve()
    run=workroot/'runs'/child_id;assert not run.exists(),'Migration targets are immutable; choose another new run.'
    manifest=read_json(parent_run/'run_manifest.json');receipt=read_json(parent_run/'latest_checkpoint.json');progress=read_json(parent_run/'progress.json')
    assert progress['flights']==receipt['flights'],'Drain and checkpoint the parent first.'
    if progress['phase']!='paused':
        # A completed curriculum review is an equally committed stopped
        # checkpoint. A concurrent evaluation may update only its UI phase.
        # Verify actual ownership and queue state instead of forging 'paused'.
        import psutil
        failure=read_json(parent_run/'failure.json')
        assert receipt['kind']=='curriculum_review' and 'failed its training-map learnability check' in failure['error']
        assert failure['flights']==receipt['flights']
        for process in psutil.process_iter(['name']):
            if 'python' not in (process.info['name'] or '').lower():continue
            try:
                cmd=process.cmdline()
                owns=any(Path(arg).resolve()==(parent_source/'python/run.py').resolve() for arg in cmd[1:] if arg.endswith('.py'))
                assert not (owns and 'train' in cmd and '--run' in cmd and cmd[cmd.index('--run')+1]==parent_run.name),'Parent learner is still active.'
            except (psutil.NoSuchProcess,psutil.AccessDenied):continue
        for part in ['queued','claimed']:
            assert not list((parent_run/'train_queue'/part).glob('*.json')),'Parent native queue is not drained.'
    assert digest(sources_at(parent_source))==manifest['source_sha256'],'Parent source drift.'
    assert sha256(parent_source/'worker_windows/uav_rl_worker.exe')==manifest['worker_sha256']
    assert sha256(ROOT/'worker_windows/uav_rl_worker.exe')==manifest['worker_sha256'],'Native safety/dynamics executable changed.'
    assert sha256(receipt['path'])==receipt['sha256']
    saved=torch.load(receipt['path'],map_location='cpu',weights_only=True)
    assert saved['config_sha256']==receipt['config_sha256']==manifest['config_sha256']
    assert saved['source_sha256']==manifest['source_sha256']==saved['state']['source_sha256']
    # Match the existing audited rollback operation: restore the best complete
    # learner while retaining the most recent Replay, map counters and budgets.
    best=read_json(parent_run/'best_model.json');seed_receipt=best['checkpoint']
    assert sha256(seed_receipt['path'])==seed_receipt['sha256']
    assert sha256(best['model'])==best['model_sha256']
    seed=torch.load(seed_receipt['path'],map_location='cpu',weights_only=True)
    assert seed['config_sha256']==saved['config_sha256'] and seed['source_sha256']==saved['source_sha256']
    # Keep all newly learned Critic/target/moments/RNG, while restoring the
    # fully validated Actor. Actor optimizer moments are reset explicitly below.
    latest_learner=copy.deepcopy(saved['learner'])
    for name in ['actor','actor_optimizer','actor_updates','campaign_guard_actor','campaign_guard_reference_inputs']:
        if name in seed['learner']:latest_learner[name]=copy.deepcopy(seed['learner'][name])
    saved['learner']=latest_learner
    critic_seed_receipt=receipt
    baseline_anchor=parent_run/'policies/parent_actor_anchor.mat'
    baseline_anchor_sha256=sha256(baseline_anchor)
    cfg=load_config();assert cfg['reward']['revisit_per_second']==2. and cfg['reward']['revisit_multiplier']==20.
    parent_cfg=read_json(parent_run/'configuration.json')
    for key in ['environment','dataset','network','reward']:assert cfg[key]==parent_cfg[key],key
    for name,expected in manifest['sources'].items():
        if name.startswith('matlab/'):assert sha256(ROOT/name)==expected,name
    cfg['migration_parent_run']=str(parent_run);cfg.pop('_sha256');cfg['_sha256']=digest(cfg)
    torch.set_num_threads(cfg['system']['torch_cpu_threads']);torch.use_deterministic_algorithms(True)
    checks=run_checks();current_sources=source_manifest();source_sha=digest(current_sources)
    learner=SAC(cfg,cfg['system']['device']);learner.load_state_dict(saved['learner'])
    expected_seed=learner.compatible_learner_state(saved['learner'])
    assert same({k:v for k,v in learner.state_dict().items() if k in expected_seed},expected_seed),'Seed tensors/moments/RNG must survive the explicit compatible expansion.'
    model_check=copy.deepcopy(learner.actor);import_actor(model_check,best['model'])
    assert same(model_check.state_dict(),learner.actor.state_dict()),'Best policy and checkpoint Actor differ.'
    del model_check
    inherited_raw={v['episode']:v for v in read_json(parent_run/'lineage/relabelled_replay.json')}

    selection=read_json(cfg['campaign']['seed_selection_file'])
    assert digest(selection)==cfg['campaign']['seed_selection_digest']
    chosen=selection['best'];assert chosen['level']=='full' and selection['test_used_for_selection'] is False
    assert sha256(chosen['model'])==chosen['model_sha256'] and chosen['bc_updates']==chosen['teacher_queries']==0
    chosen_cfg=read_json(Path(selection['run'])/'configuration.json')
    for key in ['environment','network','dataset','reward','validation']:assert cfg[key]==chosen_cfg[key],key
    assert read_json(Path(selection['run'])/'dataset_binding.json')==read_json(parent_run/'dataset_binding.json')
    original_anchor=[r for r in read_json(parent_run/'evaluations/full_baselines/results.json')['rows'] if r['arm']=='run_anchor']
    selected_rows=[r for r in read_json(Path(chosen['evaluation'])/'results.json')['rows'] if r['arm']==chosen['arm']]
    assert len(selected_rows)==256 and all(r['model_sha256']==chosen['model_sha256'] and r['audit_passed'] and r['teacher_queries']==0
        and not r['stochastic'] and r['leader_rule_planner_calls']==0 and r['task_protocol']=='global_subgoal_v1' for r in selected_rows)
    selected_gate=admissible(selected_rows,original_anchor,cfg,True);assert selected_gate['admissible'],selected_gate['problems']
    for folder in ['episodes','decisions','maps','bank','policies','evaluations','checkpoints','recovery','lineage']:(run/folder).mkdir(parents=True,exist_ok=True)
    atomic_json(run/'configuration.json',cfg)
    replay=Replay(cfg);reports=[];missing=[]
    # Reward/environment and the entire label-producing source are identical.
    # Reusing exact immutable labels avoids an unnecessary raw-physics replay.
    for name in ['reward.py','reward_v2.py','maps.py','global_obs.py','replay.py']:
        assert sha256(ROOT/'python/uav_rl'/name)==sha256(parent_source/'python/uav_rl'/name),name
    for number,key in enumerate(saved['replay']['ids'],1):
        old_path=parent_run/'episodes'/f'{key}.npz';ep=load_episode(old_path);ep.validate()
        assert ep.outcome in [1,2,3] and ep.map_key
        link(old_path,run/'episodes'/f'{key}.npz');replay.add(ep)
        raw=parent_run/'train_queue/completed'/f'{key}.mat'
        if not raw.exists() and key in inherited_raw:raw=Path(inherited_raw[key]['raw'])
        reports.append({'episode':key,'transitions':len(ep.actions),'raw':str(raw),
            'raw_sha256':sha256(raw) if raw.exists() else None,'source_episode_sha256':sha256(old_path),
            'old_reward_sum':float(ep.rewards.sum()),'new_reward_sum':float(ep.rewards.sum()),
            'labels_reused_under_identical_reward_source_and_configuration':True})
        if number%128==0:print('VERIFIED_REPLAY',number,'/',len(saved['replay']['ids']),'transitions',replay.count,flush=True)
    assert replay.count>=cfg['training']['critic_min_transitions']
    assert len(replay.eligible())>=cfg['training']['critic_min_training_episodes'] and len(replay.eligible(True))>=cfg['training']['critic_min_probe_episodes']
    replay.rng.bit_generator.state=copy.deepcopy(saved['replay']['rng'])
    # Relabeling is deterministic and must not change any learner RNG or tensor.
    assert same({k:v for k,v in learner.state_dict().items() if k in expected_seed},expected_seed)
    actor_override=chosen['model_sha256']!=best['model_sha256']
    if actor_override:
        import_actor(learner.actor,chosen['model'])
        learner.actor_updates=max(learner.actor_updates,int(chosen['actor_updates']))
    learner.actor_optimizer=torch.optim.Adam(learner.actor.parameters(),lr=cfg['learner']['actor_lr'])
    learner.alpha_optimizer=torch.optim.Adam([learner.log_alpha],lr=cfg['learner']['alpha_lr'])
    parent_guard_rejections=learner.cumulative_rejections;learner.cumulative_rejections=0
    from .mean_calibration_reuse import verify as verify_mean_calibration
    mean_reuse=verify_mean_calibration(parent_run,learner.actor,saved['replay']['ids'],cfg['campaign']['mean_calibration_flights'])
    reused_ids={r['id'] for r in mean_reuse['episodes']} if mean_reuse['reusable'] else set()
    replay.mean_policy_ids=reused_ids
    atomic_json(run/'lineage/mean_calibration_reuse.json',mean_reuse)
    # A weight average has no additional gradient updates. The update counter
    # remains that of the optimizer seed, with exact derivation recorded below.
    for group in learner.actor_optimizer.param_groups:group['lr']=cfg['learner']['actor_lr']
    learner.commit_guard_anchor();learner.fixed_guard_states=None;learner.guard_states=None
    replay.exploration_ids=set(saved['replay'].get('campaign_exploration_ids',[]))
    state=copy.deepcopy(saved['state']);parent_fl=state['flights']
    reused_count=cfg['campaign']['mean_calibration_flights'] if mean_reuse['reusable'] else 0
    state.update(campaign_stage=0,campaign_review_rotations=0,campaign_rotation_mode=False,campaign_last_failed_review=None,campaign_stage_origin=parent_fl,campaign_stage_actor_origin=state['total_actor_steps'],campaign_fresh_scheduled=0,campaign_fresh_completed=0,campaign_recent=mean_reuse['episodes'] if mean_reuse['reusable'] else [],
        mean_calibration_scheduled=reused_count,mean_calibration_completed=reused_count,
        mean_calibration_origin='verified_parent_trajectories' if mean_reuse['reusable'] else 'new_rollouts_required',
        parent_guard_rejections=parent_guard_rejections)
    state.update(source_sha256=source_sha,parent_run=str(parent_run),parent_checkpoint=receipt,
        reward_stage_start_flights=parent_fl,reward_stage_started_at=stamp(),reward_stage_last_micro=None,reward_stage_last_full=None,
        parent_actor_updates=learner.actor_updates,parent_critic_updates=learner.updates,reward_stage_total_actor_start=state['total_actor_steps'],
        parent_validation_rollbacks=state.get('parent_validation_rollbacks',0)+state['validation_rollbacks'],validation_rollbacks=0,
        critic_ready=False,critic_probe_passes=0,critic_warmup_updates=0,pending_validation=None,
        checkpoint_serial=0,policy_serial=1,phase='critic_recalibration_required')
    baseline={}
    for level in ['micro','full']:
        frozen=read_json(parent_run/'evaluations'/f'{level}_baselines/results.json')
        rows=[r for r in frozen['rows'] if r['arm']=='run_anchor']
        assert len(rows)==(32 if level=='micro' else 256)
        assert all(r['model_sha256']==baseline_anchor_sha256 for r in rows),'Cached original-anchor identity mismatch.'
        baseline[level]={'timestamp':frozen['timestamp'],'safe_success':sum(bool(r['success']) and bool(r['safe']) for r in rows),
                         'tasks':len(rows),'unsafe':sum(not bool(r['safe']) for r in rows),'model_sha256':baseline_anchor_sha256}
    state['reward_change_baseline']=baseline
    anchor=run/'policies/parent_actor_anchor.mat'
    parity=export_actor(learner.actor,run/'lineage/seed_export_parity.mat',{'seed':seed_receipt,'reward_schema':2,'bc_updates':0})
    link(baseline_anchor,anchor)
    state['parent_initial_policy']=state.get('initial_policy');state['initial_policy']=str(anchor)
    for key,item in state['bank'].items():
        link(parent_run/item['snapshot'],run/item['snapshot'])
        link(parent_run/'maps'/f'{key}.json',run/'maps'/f'{key}.json')
    # All comparator policies, including this same best run-anchor, are byte
    # identical. Dataset and native dynamics are unchanged, so retain the
    # frozen baseline trajectories/results rather than claiming a new test.
    reused=0
    for asset in (ROOT/'assets').rglob('*'):
        if asset.is_file():assert sha256(asset)==sha256(parent_source/'assets'/asset.relative_to(ROOT/'assets'))
    for level in ['micro','full']:
        src=parent_run/'evaluations'/f'{level}_baselines';dst=run/'evaluations'/f'{level}_baselines'
        link(src/'results.json',dst/'results.json')
        for path in (src/'queue/completed').iterdir():
            if path.is_file() and (path.name.endswith('.mat') or path.name.endswith('.mat.json')):
                link(path,dst/'queue/completed'/path.name);reused+=1
    for name in ['configuration.json','run_manifest.json','dataset_binding.json','metrics.jsonl','flights.jsonl']:
        if (parent_run/name).exists():link(parent_run/name,run/'lineage'/('parent_'+name))
        else:
            assert name in ['metrics.jsonl','flights.jsonl']
            (run/'lineage'/('parent_'+name)).write_text('',encoding='utf-8')
    migration={'created_at':stamp(),'parent_run':str(parent_run),'parent_checkpoint':receipt,'parent_source_sha256':manifest['source_sha256'],
        'source_sha256':source_sha,'old_config_sha256':saved['config_sha256'],'new_config_sha256':cfg['_sha256'],
        'worker_sha256':manifest['worker_sha256'],'native_safety_and_dynamics_unchanged':True,
        'critic_action_fusion_enabled':cfg['campaign'].get('critic_action_fusion_enabled',False),'legacy_critic_values_and_moments_preserved_by_zero_padding':True,
        'learner_seed_retained_before_explicit_actor_selection':True,'critic_target_optimizer_rng_retained':True,'actor_override':actor_override,'actor_optimizer_reset_for_selected_derived_weights':True,'actor_and_entropy_optimizers_fresh':True,'critic_seed_checkpoint':critic_seed_receipt,'latest_parent_critic_kept':True,'mean_calibration_reuse':mean_reuse,'selected_seed':selection,'seed_gate_recomputed':selected_gate,'actor_optimizer_lr_is_explicit_campaign_change':True,'guard_reference_rebuilt_from_successful_regular_training_episodes':True,'learner_seed_checkpoint':seed_receipt,'learner_seed_kind':'validated_actor_latest_parent_critic_fresh_actor_optimizer','recent_replay_from_latest_checkpoint':True,'selected_model_sha256':chosen['model_sha256'],'actor_frozen_for_critic_recalibration':True,
        'relabelled_episodes':len(replay.episodes),'relabelled_transitions':replay.count,'unavailable_raw_episodes_excluded':missing,
        'old_reward_labels_reused':True,'identical_reward_source_and_configuration_verified':True,'replay_sampler_rng_retained':True,'static_baseline_files_reused':reused,
        'unchanged_validated_run_anchor_reused':True,'reward_unchanged':True,'campaign_training_pipeline_changed':True,'actor_lr_explicitly_set':cfg['learner']['actor_lr'],'critic_probe_split_by_geometry':True,'unit_checks':checks,'export_parity_max_error':parity,
        'parent_validation':baseline,'parent_validation_does_not_measure_new_reward_effect':True}
    new_manifest=copy.deepcopy(manifest);new_manifest.update(created_at=stamp(),config_sha256=cfg['_sha256'],source_sha256=source_sha,sources=current_sources,
        source_root=str(ROOT),initialization='continued_own_random_lineage_with_full_validation_selected_actor',lineage_initialization='random',parent_run=str(parent_run),parent_checkpoint=receipt,
        reward_schema=2,migration=migration)
    atomic_json(run/'run_manifest.json',new_manifest);atomic_json(run/'reward_migration.json',migration)
    atomic_json(run/'lineage/relabelled_replay.json',reports)
    append_json(run/'events.jsonl',{'event':'three_day_campaign_migration','timestamp':stamp(),'parent_run':str(parent_run),
        'relabelled_episodes':len(replay.episodes),'relabelled_transitions':replay.count,'actor_frozen':True})
    seed_metric=copy.deepcopy(chosen);seed_metric.update(validation_reused_for_identical_actor=True,source_selection_run=selection['run'],new_environment_trials=False)
    append_json(run/'metrics.jsonl',seed_metric)
    child_checkpoint=checkpoint.save(run,learner,replay,state,cfg,'reward_v2_migration')
    selected_model=run/'policies/initial_selected_seed.mat';link(chosen['model'],selected_model)
    selected_best=copy.deepcopy(chosen);selected_best.update(model=str(selected_model),checkpoint=child_checkpoint,gate=selected_gate,
        actor_updates=learner.actor_updates,source_selection_run=selection['run'],seed_not_new_environment_evaluation=True)
    atomic_json(run/'best_model.json',selected_best)
    atomic_json(run/'dataset_binding.json',read_json(parent_run/'dataset_binding.json'))
    stable=run/'lineage/migration_initial_checkpoint.pt';link(child_checkpoint['path'],stable)
    atomic_json(run/'MIGRATION_COMPLETE.json',{'passed':True,'checkpoint':child_checkpoint,'permanent_checkpoint':{**child_checkpoint,'path':str(stable)},'migration':migration})
    print('MIGRATION_COMPLETE',run.name,'parent_flights',parent_fl,'episodes',len(replay.episodes),'actor_updates',learner.actor_updates,flush=True)
    return migration

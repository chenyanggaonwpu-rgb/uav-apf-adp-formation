"""Create an audited continuation stage from a supplied full checkpoint.

Preserves Actor/Critic/target/optimizer/Replay/RNG. New source/config hashes
are committed explicitly; this is not a bypass of the original resume gate.
Formal F/A1-A7 runs must instead use paired random initialization.
"""
from pathlib import Path
import argparse,sys,copy,shutil,os,json
import torch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from uav_rl.io import read_json,atomic_json,digest,sha256,source_manifest,stamp
from uav_rl.config import load_config

def main():
    p=argparse.ArgumentParser();p.add_argument('--asset',choices=['retained149','candidate158'],default='retained149')
    p.add_argument('--run',required=True);p.add_argument('--work-root',default='F:/UAV_Schema6');p.add_argument('--device',choices=['cpu','cuda'],default='cuda')
    p.add_argument('--additional-physics-steps',type=int,default=10000000);p.add_argument('--allow-rejected-branch',action='store_true');args=p.parse_args()
    assert all(c.isalnum() or c in '_-' for c in args.run) and args.additional_physics_steps>=2400
    if args.asset=='candidate158':assert args.allow_rejected_branch,'158 was rejected; explicit research-only acknowledgement is required.'
    work=Path(args.work_root).resolve();assert work.drive.upper()!='C:';assetroot=work/'checkpoint_assets'
    assert (assetroot/'EXTRACTED_AND_VERIFIED.json').exists(),'Run 09_fetch_checkpoint_assets.ps1 first.'
    asset=read_json(assetroot/'CHECKPOINT_ASSETS.json')[args.asset];raw=assetroot/asset['checkpoint_file'];assert sha256(raw)==asset['checkpoint_sha256']
    original=torch.load(raw,map_location='cpu',weights_only=True);payload=copy.deepcopy(original)
    run=work/'runs'/args.run;assert not run.exists(),'Choose an unused run name.'
    for sub in ['episodes','decisions','maps','bank','policies','evaluations','checkpoints','recovery']:(run/sub).mkdir(parents=True,exist_ok=True)
    for ep in asset['replay_episodes']:
        source=assetroot/ep['file'];assert sha256(source)==ep['sha256'];shutil.copy2(source,run/'episodes'/f'{ep["episode_id"]}.npz')
    cfg=load_config(ROOT/('config/rtx4060.json' if args.device=='cuda' else 'config/local_cpu_long.json'));cfg.pop('_sha256',None)
    cfg['training']['actor_initialization']='checkpoint';cfg['system']['device']=args.device
    history=read_json(ROOT/'provenance/MODEL_HISTORICAL_BUDGETS.json')[args.asset]
    origin=history['joint_physics_steps'];cfg['training']['maximum_physics_steps']=origin+args.additional_physics_steps
    cfg['training']['maximum_updates']=original['state']['total_gradient_steps']+200000
    cfg['training']['maximum_flights']=original['state']['flights']+200000;cfg['_sha256']=digest(cfg)
    sources=source_manifest();source_sha=digest(sources)
    state=payload['state'];state.update(source_sha256=source_sha,phase='imported_paused',pending_validation=None,
        budget_physics_steps=origin,last_full_physics=origin,last_micro_physics=origin)
    state['initial_policy']=str(run/'policies/original_global_anchor.mat')
    shutil.copy2(ROOT/'assets/reference_models/original_global_anchor.mat',state['initial_policy'])
    imported_policy=run/'policies/imported_actor.mat';shutil.copy2(assetroot/asset['actor_file'],imported_policy);assert sha256(imported_policy)==asset['actor_sha256']
    payload.update(source_sha256=source_sha,config_sha256=cfg['_sha256'],journal_offsets={name:0 for name in ['flights.jsonl','losses.jsonl','metrics.jsonl','events.jsonl']})
    cp=run/'checkpoints/m0001_imported.pt';torch.save(payload,cp)
    receipt={'path':str(cp),'sha256':sha256(cp),'kind':'audited_external_checkpoint_import','flights':state['flights'],
        'updates':payload['learner']['updates'],'actor_updates':payload['learner']['actor_updates'],'bc_updates':0,'config_sha256':cfg['_sha256']}
    atomic_json(run/'latest_checkpoint.json',receipt);atomic_json(run/'warm_anchor_checkpoint.json',receipt)
    # Baseline result rows are unchanged; only their location is different.
    for level in ['full','micro']:
        f=assetroot/f'baseline_evaluations/{level}_baselines/results.json'
        if f.exists():
            folder=run/f'evaluations/{level}_baselines';folder.mkdir(parents=True);shutil.copy2(f,folder/'results.json')
    retained=read_json(ROOT/'assets/MODEL_MANIFEST.json')['retained149'];evaluation=run/'evaluations/retained149_reference';evaluation.mkdir()
    shutil.copy2(ROOT/'provenance/retained149/results_original.json',evaluation/'results.json')
    selection=read_json(ROOT/'provenance/retained149/selection_original.json')
    fallback=(receipt if args.asset=='retained149' else {'path':str(assetroot/'retained149/checkpoint.pt'),'sha256':retained['checkpoint_sha256'],'kind':'supplied_validated_fallback'})
    selection.update(model=str(ROOT/retained['model']),checkpoint=fallback,evaluation=str(evaluation),arm='candidate')
    atomic_json(run/'best_model.json',selection)
    atomic_json(run/'configuration.json',cfg)
    manifest={'created_at':stamp(),'config_sha256':cfg['_sha256'],'source_sha256':source_sha,'sources':sources,
        'worker_sha256':sha256(ROOT/'worker_windows/uav_rl_worker.exe'),'initialization':'audited_rl_checkpoint',
        'network_hidden':[512,512,256,256],'hidden_activation':'ReLU','actor_schema':6,'action_semantics':'normalized_global_subgoal_xyz',
        'task_protocol':'global_subgoal_v1','device':args.device,'diagnostic_only':False,'parent_checkpoint_sha256':asset['checkpoint_sha256']}
    atomic_json(run/'run_manifest.json',manifest)
    proof={'imported_at':stamp(),'asset':args.asset,'source_checkpoint_sha256':asset['checkpoint_sha256'],
        'new_checkpoint':receipt,'optimizer_target_replay_rng_preserved':True,'historical_joint_steps_counted':origin,
        'additional_joint_step_budget':args.additional_physics_steps,'formal_ablation_eligible':False,
        'original_source_sha256':original['source_sha256'],'new_source_sha256':source_sha,
        'cleared_pending_validation_because_completed_original_results_are_supplied':True,
        'cuda_rng_note':'Source CUDA RNG is preserved if present. CPU-source checkpoints initialize a new CUDA RNG on first CUDA learner construction; cross-device bitwise trajectories are not claimed.'}
    atomic_json(run/'recovery/IMPORT_PROVENANCE.json',proof);(run/'PAUSE_REQUEST').touch()
    print('IMPORTED_WITHOUT_STARTING_TRAINING',run)
    print('Resume explicitly:',f'python python/run.py train --work-root "{work}" --run {args.run} --resume')
if __name__=='__main__':main()

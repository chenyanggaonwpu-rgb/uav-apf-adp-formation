"""Bounded-memory, resumable SAC with complete native fleet rollouts."""
from pathlib import Path
import collections,copy,json,os,re,time,traceback
import numpy as np
import torch
from .io import ROOT,atomic_json,append_json,read_json,sha256,digest,source_manifest,stamp
from .dataset import prepare
from .maps import generate,physical_digest
from .queue import Queue,make_job
from .networks import export_actor
from .campaign_sac import SAC
from .replay import save_episode
from .campaign_replay import Replay
from .reward import read_transition
from . import checkpoint
from .evaluate import evaluate,baselines,score,admissible


class Trainer:
    def __init__(self,cfg,workroot,run_id,resume=False,diagnostic=False):
        self.cfg=cfg;self.workroot=Path(workroot).resolve();self.run=self.workroot/'runs'/run_id;self.diagnostic=diagnostic
        assert self.workroot.drive and self.workroot.drive.upper()!='C:','Choose an explicit data drive; no automatic C: fallback.'
        assert all(c.isalnum() or c in '_-' for c in run_id)
        self.run.mkdir(parents=True,exist_ok=True)
        for name in ['episodes','decisions','maps','bank','policies','evaluations','checkpoints','recovery']:(self.run/name).mkdir(exist_ok=True)
        import msvcrt
        self.lock=(self.run/'learner.lock').open('a+b');self.lock.seek(0);self.lock.write(b'1');self.lock.flush();self.lock.seek(0)
        msvcrt.locking(self.lock.fileno(),msvcrt.LK_NBLCK,1)
        if cfg['system']['device']=='cuda':assert torch.cuda.is_available(),'CUDA is unavailable; explicitly choose the CPU profile.'
        assert cfg['training']['actor_initialization'] in ['random','checkpoint'] and (resume or cfg['training']['actor_initialization']=='random'),'Global input requires a new random Actor.'
        torch.set_num_threads(cfg['system']['torch_cpu_threads'])
        torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
        torch.use_deterministic_algorithms(True)
        self.initial=None
        self.learner=SAC(cfg,cfg['system']['device'],self.initial);self.replay=Replay(cfg)
        current_sources=source_manifest();source_sha=digest(current_sources)
        self.state={'schema':1,'flights':0,'next_job':0,'next_map':0,'policy_serial':0,'checkpoint_serial':0,'consumed':[],
            'unique_map_hashes':[],'bank':{},'critic_ready':False,'critic_probe_passes':0,'critic_warmup_updates':0,
            'total_gradient_steps':0,'total_actor_steps':0,'validation_rollbacks':0,'last_micro':0,'last_full':0,
            'source_sha256':source_sha,'started_at':stamp(),'phase':'initializing','rng':np.random.default_rng(cfg['training']['seed']+73).bit_generator.state}
        self.state.update(reward_stage_start_flights=0,parent_run=None,parent_actor_updates=0,parent_critic_updates=0,parent_validation_rollbacks=0,budget_physics_steps=0)
        latest=self.run/'latest_checkpoint.json'
        if resume:
            assert latest.exists(),'No committed checkpoint is available.'
            existing_manifest=read_json(self.run/'run_manifest.json')
            assert existing_manifest['worker_sha256']==sha256(ROOT/'worker_windows/uav_rl_worker.exe'),'Native executable changed; fork a new run.'
            self.state=checkpoint.load(read_json(latest),self.run,self.learner,self.replay,cfg)
            assert self.state['source_sha256']==source_sha,'Code changed: fork a new run instead of silently resuming.'
        else:
            assert not latest.exists() and not (self.run/'run_manifest.json').exists(),'Use --resume or a fresh run ID.'
            atomic_json(self.run/'configuration.json',cfg)
            atomic_json(self.run/'run_manifest.json',{'created_at':stamp(),'config_sha256':cfg['_sha256'],'source_sha256':source_sha,'sources':current_sources,
                'worker_sha256':sha256(ROOT/'worker_windows/uav_rl_worker.exe'),'initialization':cfg['training']['actor_initialization'],
                'baseline_local_v4_sha256':sha256(ROOT/'assets/v4_initial.mat'),'actor_schema':6,'action_semantics':'normalized_global_subgoal_xyz','task_protocol':'global_subgoal_v1','candidate_leader_rule_planner':False,'terminal_follower_planning':'shared_original','observation_scope':'full_map_geometry_plus_local_history','global_observation_schema':1,'input_size':3340,
                'network_hidden':[512,512,256,256],'hidden_activation':'ReLU',
                'actor_parameters':sum(p.numel() for p in self.learner.actor.parameters()),'new_bc_updates':0,'teacher_queries_allowed':0,
                'device':str(self.learner.device),'torch_version':str(torch.__version__),'diagnostic_only':diagnostic})
        self.rng=np.random.default_rng();self.rng.bit_generator.state=self.state['rng'];self.consumed=set(self.state['consumed'])
        self.q=Queue(self.run/'train_queue',self.workroot,cfg);self.q.recover()
        # IDs and files are never overwritten after an abrupt process loss.
        existing=[int(p.stem[1:]) for folder in ['queued','claimed','completed'] for p in (self.q.root/folder).glob('f*.json') if re.fullmatch(r'f\d+',p.stem)]
        existing += [int(p.name.split('.')[0][1:]) for p in (self.q.root/'completed').glob('f*.mat.json') if re.fullmatch(r'f\d+',p.name.split('.')[0])]
        self.state['next_job']=max([self.state['next_job']]+[i+1 for i in existing])
        self.datasets=prepare(cfg,self.workroot)
        reservation=read_json(self.datasets/'reservation.json')
        atomic_json(self.run/'dataset_binding.json',{'folder':str(self.datasets),'validation_digest':reservation['validation_digest'],
            'test_digest':reservation['test_digest'],'micro_digest':reservation['micro_digest'],'reserved_before_gradient_updates':True})
        self.heldout_hashes={c['geometry_sha256'] for split in ['validation','test'] for c in read_json(self.datasets/f'{split}.json')}
        self.q.start()
        self.policy=self.export_policy()
        if 'initial_policy' not in self.state:self.state['initial_policy']=str(self.policy)
        self.last_status=0.;self.last_checkpoint_flights=self.state['flights']
        self.recover_completed()

    def progress(self,extra=None):
        self.state['rng']=self.rng.bit_generator.state
        result={'timestamp':stamp(),'run':self.run.name,'phase':self.state['phase'],'flights':self.state['flights'],
            'maximum_flights':self.cfg['training']['maximum_flights'],'unique_maps_flown':len(self.state['unique_map_hashes']),
            'replay_transitions':self.replay.count,'replay_episodes':len(self.replay.episodes),'replay_bytes':self.replay.bytes,
            'replay_training_episodes':len(self.replay.eligible()),'replay_probe_episodes':len(self.replay.eligible(True)),
            'actor_schema':6,'task_protocol':'global_subgoal_v1','planner_decision_seconds':1.6,
            'critic_updates_in_lineage':self.learner.updates,'actor_updates_in_lineage':self.learner.actor_updates,
            'total_gradient_steps_executed':self.state['total_gradient_steps'],'total_actor_steps_executed':self.state['total_actor_steps'],
            'bc_updates':0,'teacher_queries':0,'critic_ready':self.state['critic_ready'],'critic_probe_passes':self.state['critic_probe_passes'],
            'validation_rollbacks':self.state['validation_rollbacks'],'device':str(self.learner.device),'diagnostic_only':self.diagnostic,'observation_scope':'global_map','input_size':3340,
            'free_ram_bytes':__import__('psutil').virtual_memory().available,'budget_physics_steps':self.state.get('budget_physics_steps',0),'maximum_physics_steps':self.cfg['training'].get('maximum_physics_steps')}
        if 'reward_stage_start_flights' in self.state:
            result.update(reward_schema=2,reward_stage_flights=self.state['flights']-self.state['reward_stage_start_flights'],
                reward_stage_start_flights=self.state['reward_stage_start_flights'],parent_run=self.state['parent_run'],
                critic_warmup_updates=self.state['critic_warmup_updates'],reward_change_baseline=self.state.get('reward_change_baseline'),
                reward_stage_actor_updates=self.learner.actor_updates-self.state['parent_actor_updates'],
                reward_stage_critic_updates=self.learner.updates-self.state['parent_critic_updates'],
                cumulative_validation_rollbacks=self.state.get('parent_validation_rollbacks',0)+self.state['validation_rollbacks'])
        if torch.cuda.is_available():result.update(gpu_allocated_bytes=torch.cuda.memory_allocated(),gpu_reserved_bytes=torch.cuda.memory_reserved(),gpu_peak_bytes=torch.cuda.max_memory_allocated())
        result.update(campaign_stage=self.state.get('campaign_stage',0),campaign_stage_name=self.state.get('campaign_stage_name','initializing'),campaign_deadline=self.cfg['campaign']['deadline'],cumulative_guard_rejections=self.learner.cumulative_rejections)
        result.update(mean_calibration_completed=self.state.get('mean_calibration_completed',0),mean_calibration_target=self.cfg['campaign']['mean_calibration_flights'],
            curriculum_mastery_policy='deterministic_mean')
        result.update(campaign_fresh_completed=self.state.get('campaign_fresh_completed',0),
            campaign_fresh_scheduled=self.state.get('campaign_fresh_scheduled',0),
            campaign_accepted_actor_updates_since_stage=self.state['total_actor_steps']-self.state.get('campaign_stage_actor_origin',self.state['total_actor_steps']))
        result.update(curriculum_review_rotations=self.state.get('campaign_review_rotations',0),
            curriculum_last_failed_review=self.state.get('campaign_last_failed_review'))
        if extra:result.update(extra)
        best=self.run/'best_model.json'
        if best.exists():result['best_model']=read_json(best)
        atomic_json(self.run/'progress.json',result)
        if time.monotonic()-self.last_status>20:print(json.dumps({k:result[k] for k in ['timestamp','phase','flights','replay_transitions','actor_updates_in_lineage','critic_ready']},ensure_ascii=False),flush=True);self.last_status=time.monotonic()

    def export_policy(self):
        while True:
            serial=self.state['policy_serial'];self.state['policy_serial']+=1
            path=self.run/'policies'/f'p{serial:08d}_a{self.learner.actor_updates:09d}.mat'
            if not path.exists():break
        error=export_actor(self.learner.actor,path,{'config_sha256':self.cfg['_sha256'],'source_sha256':self.state['source_sha256'],
            'actor_updates':self.learner.actor_updates,'critic_updates':self.learner.updates,'bc_updates':0,'initialization':self.cfg['training']['actor_initialization'],
            'seed':self.cfg['training']['seed'],'created_at':stamp()})
        append_json(self.run/'events.jsonl',{'event':'policy_export','time':stamp(),'path':str(path),'sha256':sha256(path),'numpy_export_max_error':error,
            'actor_updates':self.learner.actor_updates,'bc_updates':0})
        from .behavior_exploration import export_explorers
        self.exploration_policies=export_explorers(self.learner.actor,path,self.cfg['campaign']['behavior_exploration'])
        return path

    def next_job(self):
        from .campaign_curriculum import next_job
        from .behavior_exploration import decorate_job
        return decorate_job(self,next_job(self))

    def random_job(self,family_override=None,deterministic=False):
        number=self.state['next_job'];self.state['next_job']+=1;identifier=f'f{number:09d}'
        bank=[(k,v) for k,v in self.state['bank'].items() if v['reuses']<self.cfg['training']['maximum_reuses_per_map'] and (self.run/v['snapshot']).exists()]
        reuse=bool(bank and self.rng.random()>self.cfg['training']['fresh_map_probability'] and not self.diagnostic)
        if reuse:
            key,item=bank[int(self.rng.integers(len(bank)))];case=read_json(self.run/'maps'/f'{key}.json');item['reuses']+=1
            job=make_job(case,'candidate',identifier,self.policy,not deterministic,self.cfg['training']['seed']+number+900001,environment=self.cfg['environment']);job['resume_file']=str(self.run/item['snapshot'])
        else:
            for _ in range(100):
                n=self.state['next_map'];self.state['next_map']+=1
                family=['single_window','double_window'][number%2] if self.diagnostic else (family_override or self.cfg['dataset']['families'][number%len(self.cfg['dataset']['families'])])
                try:case=generate(self.cfg,'train',n,family)
                except RuntimeError as e:
                    append_json(self.run/'events.jsonl',{'event':'map_generation_rejection','number':n,'family':family,'reason':str(e)});continue
                if case['geometry_sha256'] not in self.heldout_hashes:break
            else:raise RuntimeError('Training map generation failed repeatedly; inspect generator evidence.')
            if self.diagnostic:
                case.update(context='global_planning',noise_profile='clean',noise_std=0.,wind_bias=[0.,0.,0.],random_acceleration_max=0.,sensor_delay_steps=0,ray_noise_std=0.,ray_dropout_probability=0.)
                case['case_sha256']=digest({k:v for k,v in case.items() if k!='case_sha256'})
            atomic_json(self.run/'maps'/f'{case["id"]}.json',case)
            job=make_job(case,'candidate',identifier,self.policy,not deterministic,self.cfg['training']['seed']+number+900001,environment=self.cfg['environment'])
            # Full episodes always start at t=0; snapshots are exercised separately in QA.
        job['noise_hold_seconds']=self.cfg['training']['rl_noise_hold_seconds' if self.state['critic_ready'] else 'warmup_noise_hold_seconds']
        return job

    def ingest(self,job):
        if job['id'] in self.consumed:return 0
        path=self.q.result(job['id']);ep,row,reward=read_transition(path,self.cfg,job['scene'],self.run/'decisions'/f'{job["id"]}.npz')
        from .campaign_curriculum import note_outcome
        row['behavior_exploration']=bool(job.get('behavior_exploration',False))
        row['campaign_rehearsal']=bool(job.get('campaign_rehearsal',False))
        row['mean_calibration']=bool(job.get('mean_calibration',False))
        row['campaign_fresh']=bool(job.get('campaign_fresh',False))
        if row['campaign_fresh']:self.state['campaign_fresh_completed']=self.state.get('campaign_fresh_completed',0)+1
        if ep is not None and not bool(row['stochastic']):self.replay.mean_policy_ids.add(ep.episode_id)
        if row['mean_calibration']:self.state['mean_calibration_completed']=self.state.get('mean_calibration_completed',0)+1
        if ep is not None and row['behavior_exploration']:self.replay.exploration_ids.add(ep.episode_id)
        note_outcome(self,row)
        assert row['split']=='train' and row['teacher_queries']==0
        if ep is not None:save_episode(self.run/'episodes'/f'{ep.episode_id}.npz',ep);self.replay.add(ep)
        record={'timestamp':stamp(),**row,**reward,'geometry_sha256':job['scene']['geometry_sha256'],'model_sha256':job['model_sha256'],
            'raw_result':str(path),'raw_sha256':sha256(path),'raw_bytes':path.stat().st_size}
        self.state['budget_physics_steps']=self.state.get('budget_physics_steps',0)+int(reward.get('actual_new_physics_steps',row['steps']))
        append_json(self.run/'flights.jsonl',record);self.consumed.add(job['id']);self.state['consumed'].append(job['id']);self.state['flights']+=1
        if job['scene']['geometry_sha256'] not in self.state['unique_map_hashes']:self.state['unique_map_hashes'].append(job['scene']['geometry_sha256'])
        if not (row['success'] and row['safe']):
            representatives=self.state.setdefault('representative_failures',{}).setdefault(row['group'],[])
            representatives.append(job['id']);del representatives[:-self.cfg['retention']['maximum_representative_failures_per_family']]
        if row.get('capture_exists') and row.get('capture_file'):
            snapshot=Path(row['capture_file']);key=job['scene']['id']
            self.state['bank'][key]={'snapshot':snapshot.relative_to(self.run).as_posix(),'group':row['group'],'reuses':0}
            same=[(k,v) for k,v in self.state['bank'].items() if v['group']==row['group']]
            for old,item in same[:-32]:
                old_path=(self.run/item['snapshot']).resolve();assert old_path.is_relative_to((self.run/'bank').resolve())
                old_path.unlink(missing_ok=True);self.state['bank'].pop(old)
        return len(ep.actions) if ep is not None else 0

    def recover_completed(self):
        from scipy.io import loadmat
        pending=[read_json(p) for folder in ['queued','claimed'] for p in (self.q.root/folder).glob('f*.json')]
        if pending:
            self.state['phase']='recovering_queued_rollouts'
            while not all(self.q.ready(j['id']) for j in pending):self.q.check(pending);self.progress();time.sleep(1)
        for meta in sorted((self.q.root/'completed').glob('f*.mat.json')):
            key=meta.name.split('.')[0]
            if key in self.consumed:continue
            mat=meta.with_name(key+'.mat')
            if mat.exists():
                from .verify import native_dict
                job=native_dict(loadmat(mat,variable_names=['job'],simplify_cells=True)['job']);self.ingest(job)

    def train_updates(self,new_transitions):
        cfg=self.cfg['training']
        if (self.replay.count<cfg['critic_min_transitions']
            or len(self.replay.episodes)<cfg['critic_min_episodes']
            or len(self.replay.eligible())<cfg['critic_min_training_episodes']
            or len(self.replay.eligible(True))<cfg['critic_min_probe_episodes']
            or len(self.state['unique_map_hashes'])<cfg['critic_min_unique_maps']):return
        self.state['phase']='reinforcement_learning' if self.state['critic_ready'] else 'critic_warmup_actor_frozen'
        self.replay.focus_families=self.cfg['campaign']['curriculum'][self.state.get('campaign_stage',0)]['families']
        calibration_complete=self.state.get('mean_calibration_completed',0)>=self.cfg['campaign']['mean_calibration_flights']
        if calibration_complete:self.learner.set_guard_panel(self.replay.guard_panel(64))
        else:self.state['phase']='deterministic_train_calibration_actor_frozen'
        steps=min(cfg['maximum_updates_per_flight']*self.cfg['system']['workers'],int(new_transitions*cfg['updates_per_transition']))
        if not self.state['critic_ready']:steps=max(steps,min(self.cfg['system'].get('warmup_updates_per_cohort',1000),cfg['critic_max_warmup_updates']-self.state['critic_warmup_updates']))
        for _ in range(steps):
            result=self.learner.update(self.replay.sample(),self.state['critic_ready'] and calibration_complete);self.state['total_gradient_steps']+=1
            self.state['total_actor_steps']+=int(result['actor_step'])
            if not self.state['critic_ready']:self.state['critic_warmup_updates']+=1
            result.update(executed_update_id=self.state['total_gradient_steps'],flights=self.state['flights'],timestamp=stamp())
            if self.learner.updates%10==0:append_json(self.run/'losses.jsonl',result)
            if self.learner.updates%cfg['critic_probe_every_updates']==0:
                probe=self.learner.probe(self.replay.sample(self.cfg['training']['critic_probe_batch_size'],probe=True));passed=probe['finite'] and probe['td_rmse']<=cfg['critic_probe_max_td_rmse'] and probe['maximum_q_abs']<=cfg['critic_probe_max_q_abs'] and probe['terminal_checks_passed']
                self.state['critic_probe_passes']=self.state['critic_probe_passes']+1 if passed else 0
                append_json(self.run/'events.jsonl',{'event':'critic_probe','passed':passed,'updates':self.learner.updates,**probe})
                self.state['critic_bad_probes']=0 if passed else self.state.get('critic_bad_probes',0)+1
                if self.state['critic_ready'] and self.state['critic_bad_probes']>=2:
                    self.state['critic_ready']=False;self.state['critic_probe_passes']=0
                    append_json(self.run/'events.jsonl',{'event':'actor_refrozen_for_terminal_probe','updates':self.learner.updates})
                if not self.state['critic_ready'] and self.state['critic_warmup_updates']>=cfg['critic_warmup_updates'] and self.state['critic_probe_passes']>=cfg['critic_probe_required_passes']:
                    self.state['critic_ready']=True;receipt=self.save_checkpoint('critic_ready')
                    if not (self.run/'warm_anchor_checkpoint.json').exists():atomic_json(self.run/'warm_anchor_checkpoint.json',receipt)
                    break
            if not self.state['critic_ready'] and self.state['critic_warmup_updates']>=cfg['critic_max_warmup_updates']:
                raise RuntimeError('Critic warmup failed its independent training-episode probes; actor remains unchanged. Review reward/Q diagnostics before continuing.')
            if self.state['total_gradient_steps']>=cfg['maximum_updates']:break
            if self.learner.updates%100==0:self.progress({'latest_loss':result})

    def save_checkpoint(self,kind='latest'):
        self.state['rng']=self.rng.bit_generator.state;self.state['consumed']=sorted(self.consumed)
        receipt=checkpoint.save(self.run,self.learner,self.replay,self.state,self.cfg,kind);self.last_checkpoint_flights=self.state['flights'];checkpoint.trim(self.run,self.cfg['retention']['recent_full_checkpoints'])
        return receipt

    def rollback(self,reason):
        best=self.run/'best_model.json'
        receipt=read_json(best)['checkpoint'] if best.exists() else read_json(self.run/'warm_anchor_checkpoint.json')
        assert sha256(receipt['path'])==receipt['sha256']
        old=torch.load(receipt['path'],map_location='cpu',weights_only=True)
        current=self.learner.state_dict();retained_critic_updates=self.learner.updates
        for name in ['actor','actor_optimizer','log_alpha','alpha_optimizer','campaign_guard_actor','campaign_guard_reference_inputs']:
            if name in old['learner']:current[name]=copy.deepcopy(old['learner'][name])
        current['actor_updates']=old['learner']['actor_updates']
        self.learner.load_state_dict(current);self.learner.commit_guard_anchor()
        assert self.learner.updates==retained_critic_updates
        self.state.update(critic_ready=False,critic_probe_passes=0,critic_bad_probes=0,critic_warmup_updates=0)
        self.state['guard_micro_score']=old['state'].get('guard_micro_score')
        self.state['validation_rollbacks']+=1
        for group in self.learner.actor_optimizer.param_groups:group['lr']=max(self.cfg['learner']['minimum_actor_lr'],group['lr']*.5**self.state['validation_rollbacks'])
        append_json(self.run/'events.jsonl',{'event':'validation_rollback','timestamp':stamp(),'reason':reason,'restored':receipt,
            'rollbacks':self.state['validation_rollbacks'],'actor_entropy_optimizers_restored':True,'critic_target_optimizer_and_rng_learning_retained':True,'critic_recalibration_required':True,'recent_replay_retained':True})
        if self.state['validation_rollbacks']>=self.cfg['training']['maximum_validation_rollbacks']:raise RuntimeError('Repeated validation regressions; paused for review instead of spending the remaining training budget blindly.')
        self.policy=self.export_policy()

    def validate(self,full):
        self.q.stop();level='full' if full else 'micro';cases=read_json(self.datasets/('validation.json' if full else 'micro.json'))
        if self.diagnostic:cases=cases[:self.cfg['validation'].get('diagnostic_case_limit',len(cases))]
        self.state['phase']='validation_'+level;self.progress()
        pending=self.state.get('pending_validation')
        if pending:
            assert pending['full']==full;self.policy=Path(pending['model']);destination=Path(pending['destination'])
            receipt=read_json(self.run/'latest_checkpoint.json')
        else:
            self.policy=self.export_policy();destination=self.run/'evaluations'/f'{level}_f{self.state["flights"]:09d}_p{self.state["policy_serial"]:08d}'
            self.state['pending_validation']={'full':full,'model':str(self.policy),'model_sha256':sha256(self.policy),'destination':str(destination)}
            receipt=self.save_checkpoint('pre_validation')
        reference_path=self.run/'evaluations'/f'{level}_baselines'
        if (reference_path/'results.json').exists():baseline=read_json(reference_path/'results.json')['rows']
        else:
            models=baselines()
            if self.initial is None:models['run_anchor']=Path(self.state['initial_policy'])
            baseline,_=evaluate(cases,models,reference_path,self.workroot,self.cfg,self.progress)
        rows,summary=evaluate(cases,{'candidate':self.policy},destination,self.workroot,self.cfg,self.progress)
        anchor_arm='v4_anchor' if self.initial is not None else 'run_anchor'
        anchor=[r for r in baseline if r['arm']==anchor_arm];best_path=self.run/'best_model.json'
        if full and best_path.exists():
            best=read_json(best_path);reference=read_json(Path(best['evaluation'])/'results.json')['rows']
            if best['arm']!='candidate':reference=anchor
            else:reference=[r for r in reference if r['arm']=='candidate']
        else:reference=anchor
        existing=destination/'selection.json'
        if existing.exists():
            selection=read_json(existing);assert selection['model_sha256']==sha256(self.policy);gate=selection['gate']
        else:
            gate=admissible(rows,reference,self.cfg,full);selection={'timestamp':stamp(),'level':level,'model':str(self.policy),'model_sha256':sha256(self.policy),
                'checkpoint':receipt,'actor_updates':self.learner.actor_updates,'bc_updates':0,'gate':gate,'teacher_queries':0,'evaluation':str(destination),'arm':'candidate'}
            if full:
                original_gate=admissible(rows,anchor,self.cfg,True);gate['original_anchor_gate']=original_gate
                if not original_gate['admissible']:
                    gate['admissible']=False;gate['improved']=False
                    gate['problems'].extend('original_anchor_'+v for v in original_gate['problems'])
            atomic_json(existing,selection)
        if not full and gate['admissible']:
            current=gate['candidate'];prior=self.state.get('guard_micro_score') or gate['reference']
            nonreg=(current['safe_success']>=prior['safe_success'] and current['unsafe']<=prior['unsafe']
                    and current['failure_counted_120_mean']<=prior['failure_counted_120_mean']+.5
                    and all(current['by_family'][g]['safe_success']>=v['safe_success'] for g,v in prior['by_family'].items()))
            if nonreg:
                self.learner.commit_guard_anchor();self.state['guard_micro_score']=current
                append_json(self.run/'events.jsonl',{'event':'micro_guard_anchor_committed','timestamp':stamp(),'model_sha256':sha256(self.policy),'safe_success':current['safe_success']})
        append_json(self.run/'metrics.jsonl',selection)
        self.state['last_full' if full else 'last_micro']=self.state['flights']
        self.state['last_full_physics' if full else 'last_micro_physics']=self.state.get('budget_physics_steps',0)
        if full:self.state['last_micro_physics']=self.state.get('budget_physics_steps',0)
        self.state['reward_stage_last_full' if full else 'reward_stage_last_micro']=self.state['flights']
        if full:
            if gate['improved']:
                self.learner.commit_guard_anchor();selection['checkpoint']=self.save_checkpoint('validated_guard_anchor');atomic_json(best_path,selection)
            elif not best_path.exists():
                anchor_checkpoint=read_json(self.run/('warm_anchor_checkpoint.json' if (self.run/'warm_anchor_checkpoint.json').exists() else 'latest_checkpoint.json'))
                anchor_model=ROOT/'assets/v4_initial.mat' if self.initial is not None else Path(self.state['initial_policy'])
                atomic_json(best_path,{'timestamp':stamp(),'level':'full','model':str(anchor_model),'model_sha256':sha256(anchor_model),
                    'checkpoint':anchor_checkpoint,'actor_updates':anchor_checkpoint['actor_updates'],'bc_updates':0,'evaluation':str(reference_path),'arm':anchor_arm,'gate':{'candidate':score(anchor),'improved':False}})
        if not gate['admissible']:self.rollback(gate['problems'])
        self.state['pending_validation']=None
        self.save_checkpoint('post_validation');self.q.start()

    def prune(self):
        # Keep all evaluated weights and all replay/last-checkpoint episodes.
        # Raw training traces are a bounded ring; their receipts/hashes remain.
        budget=self.cfg['retention']['raw_train_gib']*1024**3;files=sorted((self.q.root/'completed').glob('f*.mat'),key=lambda p:p.name)
        total=sum(p.stat().st_size for p in files)
        representatives={key for values in self.state.get('representative_failures',{}).values() for key in values}
        for path in files[:-128]:
            if total<=budget:break
            if path.stem in self.consumed and path.stem not in representatives:
                size=path.stat().st_size;path.unlink();total-=size
        pinned={str(self.policy.resolve()),str(Path(self.state['initial_policy']).resolve())}
        pinned.update(str(p.resolve()) for p in getattr(self,'exploration_policies',[]))
        for path in (self.run/'evaluations').glob('*/selection.json'):pinned.add(str(Path(read_json(path)['model']).resolve()))
        best_model=self.run/'best_model.json'
        if best_model.exists():pinned.add(str(Path(read_json(best_model)['model']).resolve()))
        for folder in ['queued','claimed']:
            for path in (self.q.root/folder).glob('*.json'):
                model=read_json(path).get('model')
                if model:pinned.add(str(Path(model).resolve()))
        policies=sorted((self.run/'policies').glob('*.mat'))
        for path in policies[:-8]:
            if str(path.resolve()) not in pinned:path.unlink()

    def loop(self):
        try:
            if not (self.run/'latest_checkpoint.json').exists():self.save_checkpoint('initial')
            if self.state.get('pending_validation'):self.validate(self.state['pending_validation']['full'])
            while self.state['flights']<self.cfg['training']['maximum_flights'] and self.state['total_gradient_steps']<self.cfg['training']['maximum_updates']:
                cutoff=self.cfg['campaign'].get('training_cutoff_epoch')
                if cutoff is not None and time.time()>=cutoff:self.state['phase']='campaign_training_cutoff';break
                if (self.run/'PAUSE_REQUEST').exists():self.state['phase']='paused';break
                import psutil
                if psutil.disk_usage(str(self.workroot)).free<8*1024**3:self.state['phase']='paused_low_disk_space';break
                if psutil.virtual_memory().available<self.cfg['system'].get('minimum_free_ram_gib',.25)*1024**3:
                    self.state['phase']='waiting_for_free_memory';self.progress();time.sleep(5);continue
                remaining=self.cfg['training'].get('maximum_physics_steps',10**30)-self.state.get('budget_physics_steps',0)
                slots=int(remaining//2400)
                if slots<1:
                    self.state['phase']='physical_budget_complete'
                    if self.state.get('last_full_physics',-1)!=self.state.get('budget_physics_steps',0):self.validate(True)
                    self.state['phase']='physical_budget_complete';break
                self.state['phase']='collecting_real_native_rollouts';jobs=[self.next_job() for _ in range(min(self.cfg['system']['workers'],slots,self.cfg['training']['maximum_flights']-self.state['flights']))]
                for j in jobs:self.q.submit(j)
                while not all(self.q.ready(j['id']) for j in jobs):self.q.check(jobs);self.progress({'inflight_jobs':[j['id'] for j in jobs]});time.sleep(1)
                new=sum(self.ingest(j) for j in jobs);self.train_updates(new)
                f=self.state['flights'];ready=self.state['critic_ready'] and self.learner.actor_updates>0
                if f%self.cfg['training']['policy_export_every_flights']<self.cfg['system']['workers']:self.policy=self.export_policy()
                origin=self.state.get('reward_stage_start_flights',0)
                last_full=self.state.get('reward_stage_last_full');last_micro=self.state.get('reward_stage_last_micro')
                due_full=(f-origin>=self.cfg['validation']['stage_first_full_after_flights']) if last_full is None else f-last_full>=self.cfg['validation']['full_every_flights']
                due_micro=(f-origin>=self.cfg['validation']['stage_first_micro_after_flights']) if last_micro is None else f-last_micro>=self.cfg['validation']['micro_every_flights']
                physical_schedule=not self.diagnostic and 'full_every_physics_steps' in self.cfg['validation']
                if physical_schedule:
                    steps=self.state.get('budget_physics_steps',0)
                    due_full=steps-self.state.get('last_full_physics',0)>=self.cfg['validation']['full_every_physics_steps']
                    due_micro=steps-self.state.get('last_micro_physics',0)>=self.cfg['validation']['micro_every_physics_steps']
                if (ready or physical_schedule) and due_full:self.validate(True)
                elif (ready or physical_schedule) and due_micro:self.validate(False)
                if f-self.last_checkpoint_flights>=self.cfg['system'].get('checkpoint_every_flights',32):self.save_checkpoint();self.prune()
                self.progress()
            else:self.state['phase']='training_budget_complete_ready_for_independent_test'
            self.save_checkpoint('paused_or_complete');self.progress()
        except BaseException:
            self.state['phase']='needs_attention';self.progress({'error':traceback.format_exc()})
            atomic_json(self.run/'failure.json',{'timestamp':stamp(),'error':traceback.format_exc(),'flights':self.state['flights']})
            raise
        finally:
            if self.q.workers:self.q.stop(force=True)
            self.lock.close()

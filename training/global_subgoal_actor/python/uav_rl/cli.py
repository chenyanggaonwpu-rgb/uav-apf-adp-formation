import argparse,copy,json,os,sys
from pathlib import Path
from .io import ROOT,read_json,atomic_json,digest,stamp
from .config import load_config


def effective(args,command):
    path=Path(args.work_root)/'runs'/args.run/'configuration.json'
    if args.resume and path.exists():return read_json(path)
    config=Path(args.config) if args.config else ROOT/'config'/('smoke.json' if command=='smoke' else 'rtx4060.json')
    cfg=load_config(config);cfg.pop('_sha256',None)
    if args.device:cfg['system']['device']=args.device
    if args.workers:cfg['system']['workers']=args.workers
    if args.initialization:cfg['training']['actor_initialization']=args.initialization
    if args.seed is not None:cfg['training']['seed']=args.seed
    if command in ['train','smoke'] and not args.workers:
        import psutil
        if psutil.virtual_memory().available<4*1024**3:cfg['system']['workers']=1
    cfg['_sha256']=digest(cfg);return cfg


def main():
    parser=argparse.ArgumentParser(description='Full-mission global-subgoal Actor SAC; local CPU long training is explicitly supported.')
    parser.add_argument('command',choices=['preflight','selftest','prepare','train','smoke','status','monitor','pause','test','freeze','report'])
    parser.add_argument('--work-root',default='F:/UAV_Schema6');parser.add_argument('--run',default='schema6_s20261110')
    parser.add_argument('--config');parser.add_argument('--device',choices=['cpu','cuda']);parser.add_argument('--workers',type=int,choices=[1,2,3])
    parser.add_argument('--initialization',choices=['random','checkpoint']);parser.add_argument('--seed',type=int);parser.add_argument('--resume',action='store_true')
    parser.add_argument('--diagnostic',action='store_true');parser.add_argument('--port',type=int,default=8772);parser.add_argument('--open',action='store_true')
    parser.add_argument('--console',action='store_true');parser.add_argument('--output');parser.add_argument('--pause-after-flights',type=int)
    args=parser.parse_args();root=Path(args.work_root).resolve();run=root/'runs'/args.run
    if root.drive.upper()=='C:':raise ValueError('Select an explicit data drive; new work is not placed on C: by default.')
    os.environ.setdefault('PYTHONDONTWRITEBYTECODE','1');os.environ.setdefault('CUBLAS_WORKSPACE_CONFIG',':4096:8')
    for name in ['TMP','TEMP','PIP_CACHE_DIR','MPLCONFIGDIR']:
        destination=root/('plot_cache' if name=='MPLCONFIGDIR' else 'pip_cache' if name=='PIP_CACHE_DIR' else 'temp')
        destination.mkdir(parents=True,exist_ok=True);os.environ[name]=str(destination)
    if args.command=='status':print(json.dumps(read_json(run/'progress.json'),ensure_ascii=False,indent=2));return
    if args.command=='monitor':
        from .monitor import serve,watch
        return watch(run) if args.console else serve(run,args.port,args.open)
    if args.command=='pause':
        assert (run/'run_manifest.json').exists();(run/'PAUSE_REQUEST').touch();print('PAUSE_REQUESTED: the active full-fleet jobs finish before checkpointing.');return
    if args.command in ['test','freeze','report']:
        from .finalize import independent_test,freeze,report
        if args.command=='test':
            if not args.diagnostic:
                panel=read_json(root/'FINAL_MODEL_PANEL.json');selected=read_json(run/'best_model.json')
                assert any(x['sha256']==selected['model_sha256'] for x in panel['models']),'Selected model is not in the locked TEST panel.'
                assert panel['test_digest']==read_json(run/'dataset_binding.json')['test_digest']
            def progress(value):atomic_json(run/'test_progress.json',{'timestamp':stamp(),**value})
            result=independent_test(run,root,args.diagnostic,progress);report(run);print(json.dumps(result['acceptance'],ensure_ascii=False));return
        if args.command=='freeze':
            destination=Path(args.output) if args.output else root/'frozen'/f'{args.run}_{stamp().replace(":","").replace(" ","_")}'
            print(json.dumps(freeze(run,destination,args.diagnostic),ensure_ascii=False,indent=2));return
        print(report(run));return
    cfg=effective(args,args.command);diagnostic=args.diagnostic or args.command=='smoke'
    if args.command in ['train','smoke'] and diagnostic:
        assert cfg['training']['maximum_flights']<=32 and cfg['training']['maximum_updates']<=2000,'A diagnostic cannot silently launch a long CPU training run.'
    if args.command in ['preflight','selftest']:
        from .verify import preflight,learner_check
        print(json.dumps(preflight(cfg,root,diagnostic),ensure_ascii=False,indent=2))
        if args.command=='selftest':print(json.dumps(learner_check(cfg,root),ensure_ascii=False,indent=2))
        return
    if args.command=='prepare':
        from .dataset import prepare
        print(prepare(cfg,root));return
    from .train import Trainer
    if args.resume:(run/'PAUSE_REQUEST').unlink(missing_ok=True)
    trainer=Trainer(cfg,root,args.run,args.resume,diagnostic)
    if args.pause_after_flights is not None:
        original_progress=trainer.progress
        def with_pause(extra=None):
            original_progress(extra)
            if trainer.state['flights']>=args.pause_after_flights:(run/'PAUSE_REQUEST').touch()
        trainer.progress=with_pause
    trainer.loop()


if __name__=='__main__':main()

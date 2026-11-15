"""Deterministic native evaluation of a named, SHA-verified reference model."""
from pathlib import Path
import argparse,sys,json,os
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from uav_rl.io import read_json,atomic_json,digest,sha256,source_manifest
from uav_rl.config import load_config
from uav_rl.dataset import prepare
from uav_rl.evaluate import evaluate

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--model',choices=['retained149','candidate158'],required=True)
    parser.add_argument('--split',choices=['validation','micro','test'],default='validation');parser.add_argument('--limit',type=int)
    parser.add_argument('--workers',type=int,default=1);parser.add_argument('--work-root',default='F:/UAV_Schema6');parser.add_argument('--audit',action='store_true')
    args=parser.parse_args();work=Path(args.work_root).resolve();assert work.drive.upper()!='C:'
    for name in ['temp','plot_cache']:(work/name).mkdir(parents=True,exist_ok=True)
    os.environ.update(TEMP=str(work/'temp'),TMP=str(work/'temp'),MPLCONFIGDIR=str(work/'plot_cache'),PYTHONDONTWRITEBYTECODE='1')
    cfg=load_config(ROOT/'config/local_cpu_long.json');cfg.pop('_sha256',None);cfg['system']['workers']=args.workers;cfg['_sha256']=digest(cfg)
    entry=read_json(ROOT/'assets/MODEL_MANIFEST.json')[args.model];model=ROOT/entry['model'];assert sha256(model)==entry['sha256']
    folder=prepare(cfg,work);cases=read_json(folder/f'{args.split}.json')
    if args.split=='test':
        panel=read_json(work/'FINAL_MODEL_PANEL.json');assert any(row['sha256']==entry['sha256'] for row in panel['models'])
        assert panel['test_digest']==read_json(folder/'reservation.json')['test_digest']
    if args.limit is not None:assert 0<args.limit<=len(cases);cases=cases[:args.limit]
    worker_hash=sha256(ROOT/'worker_windows/uav_rl_worker.exe')
    output=work/'reproductions'/f'{args.model}_{args.split}_{len(cases)}_{worker_hash[:10]}'
    request={'model_sha256':entry['sha256'],'case_digest':digest(cases),'worker_sha256':worker_hash,'source_sha256':digest(source_manifest()),'engineering_subset':args.limit is not None}
    if (output/'request.json').exists():assert read_json(output/'request.json')==request,'Use a new output root for changed code.'
    else:atomic_json(output/'request.json',request)
    def progress(v):atomic_json(output/'progress.json',v)
    rows,summary=evaluate(cases,{'candidate':model},output,work,cfg,progress)
    if args.audit:
        from uav_rl.verify import audit_trace
        checks=[audit_trace(p) for p in (output/'queue/completed').glob('*.mat')]
        atomic_json(output/'independent_physics_audit.json',{'passed':all(c['passed'] for c in checks),'checks':checks})
    print(json.dumps({'scope':'engineering subset' if args.limit else args.split,'model':args.model,'summary':summary,'output':str(output)},ensure_ascii=False,indent=2))
if __name__=='__main__':main()

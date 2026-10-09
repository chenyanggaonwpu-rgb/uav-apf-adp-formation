"""Freeze and execute postprocessing after the numerical protocol completes."""
from __future__ import annotations
import argparse, hashlib, importlib.util, json, os, shutil, subprocess, time
from pathlib import Path

if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('study',type=Path)
    parser.add_argument('--repair',help='Separate postprocessing snapshot name; preserve the original snapshot')
    args=parser.parse_args()
    if args.repair:
        assert args.repair.startswith('postprocessing_repair') and Path(args.repair).name==args.repair
    study=args.study.resolve(); root=Path(__file__).resolve().parents[1]
    print('POSTPROCESS_WAITING_FOR_NUMERICAL_COMPLETION',flush=True)
    while not (study/'execution_complete.json').exists(): time.sleep(2)
    post=study/(args.repair or 'postprocessing_snapshot'); post.mkdir(exist_ok=True)
    names=['audit_clearance_study3d.m','plot_clearance_study3d.m','illustrate_clearance_study3d.m','analyze_clearance_study.py','postprocess_clearance_study.py','execute_clearance_protocol.py','check_clearance_baseline.py']
    hashes={}
    for name in names:
        source=root/'experiments3d'/name; target=post/name
        if target.exists(): assert target.read_bytes()==source.read_bytes(), 'Postprocessing snapshot differs; preserve old snapshot and record a repair separately'
        else: shutil.copy2(source,target)
        hashes[name]=hashlib.sha256(target.read_bytes()).hexdigest()
    hashfile=(args.repair+'_sha256.json') if args.repair else 'postprocessing_source_sha256.json'
    (study/hashfile).write_text(json.dumps(hashes,indent=2),encoding='utf-8')
    runtime=root/'.runtime/margin_study_20261006'/(args.repair or 'postprocessing'); runtime.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy(); env['MATLAB_PREFDIR']=str(runtime/'prefs'); Path(env['MATLAB_PREFDIR']).mkdir(exist_ok=True)
    code=(f"cd('{root.as_posix()}'); addpath('{post.as_posix()}'); "
          f"audit_clearance_study3d('{study.as_posix()}'); "
          f"plot_clearance_study3d('{study.as_posix()}'); "
          f"illustrate_clearance_study3d('{study.as_posix()}');")
    info=subprocess.STARTUPINFO(); info.dwFlags|=subprocess.STARTF_USESHOWWINDOW; info.wShowWindow=0
    with (runtime/'stdout.log').open('wb') as out:
        process=subprocess.Popen(['C:/Program Files/MATLAB/R2024a/bin/matlab.exe','-singleCompThread','-batch',code,
                    '-logfile',str(runtime/'matlab.log')],cwd=root,env=env,stdout=out,stderr=subprocess.STDOUT,
                    startupinfo=info,creationflags=subprocess.CREATE_NO_WINDOW)
        last=0; started=time.monotonic()
        while process.poll() is None:
            time.sleep(2); elapsed=time.monotonic()-started
            if elapsed-last>=30:
                print(f'POSTPROCESS_MATLAB_RUNNING elapsed={elapsed:.0f}s',flush=True); last=elapsed
        assert process.returncode==0, 'MATLAB postprocessing failed; inspect retained runtime log'
    frozen=study/'source_snapshot/experiments3d/clearance_study.py'
    spec=importlib.util.spec_from_file_location('integrity_controller',frozen); module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    module.verify(study)
    spec=importlib.util.spec_from_file_location('analysis_report',post/'analyze_clearance_study.py'); report=importlib.util.module_from_spec(spec); spec.loader.exec_module(report)
    report.build(study)
    (study/'postprocessing_complete.json').write_text(json.dumps(dict(passed=True,completedLocal=time.strftime('%Y-%m-%d %H:%M:%S'),
        activeSnapshot=post.name,sourceHashFile=hashfile,numericalSnapshotUnchanged=True),indent=2),encoding='utf-8')
    print('ALL_POSTPROCESSING_COMPLETE',flush=True)

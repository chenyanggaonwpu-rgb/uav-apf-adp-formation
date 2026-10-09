"""Additional execution slots; original numerical snapshot and jobs stay frozen.

Only complete map checkpoints are published. Windows rename refuses to replace
an existing destination. The original owner performs the canonical raw audit.
"""
from __future__ import annotations
import argparse,difflib,json,os,shutil,time
from pathlib import Path
import margin_grid49 as m

def prepare(study):
    assert os.name=='nt'
    m.verify(study);source=study/'aux_runtime_source';source.mkdir(exist_ok=True)
    original=(study/'source_snapshot/experiments3d/run_margin_grid49_3d.m').read_text(encoding='utf-8')
    code=original.replace('function run_margin_grid49_3d(studyDir,workerIndex)',
                          'function run_margin_grid49_aux3d(studyDir,workerIndex,seedList)')
    code=code.replace('selected=manifest(mod(manifest.seed-2001,protocol.nWorkers)+1==workerIndex,:);',
        "selected=sortrows(manifest(ismember(manifest.seed,seedList),:),'seed','descend');")
    code=code.replace("directory=fullfile(studyDir,'workers',sprintf('worker_%02d',workerIndex));",
        "directory=fullfile(studyDir,'auxiliary_runs',sprintf('aux_%02d',workerIndex));")
    code=code.replace("trialDir=fullfile(directory,'trials');", "trialDir=fullfile(directory,'pending_results');")
    code=code.replace('m=selected(index,:);frozen=load(fullfile(studyDir,char(m.mapFile)));',
        """m=selected(index,:);
        owner=mod(m.seed-2001,protocol.nWorkers)+1;
        ownerTrials=fullfile(studyDir,'workers',sprintf('worker_%02d',owner),'trials');
        if ~isempty(dir(fullfile(ownerTrials,sprintf('*_s%06d.mat',m.seed))))
            fprintf('AUX_SKIP_ALREADY_STARTED seed=%d\\n',m.seed);continue;
        end
        mapAborted=false;
        frozen=load(fullfile(studyDir,char(m.mapFile)));""")
    code=code.replace('for a=1:numel(protocol.pathValues)\n',
        """for a=1:numel(protocol.pathValues)
            if ~isempty(dir(fullfile(ownerTrials,sprintf('*_s%06d.mat',m.seed))))
                mapAborted=true;break;
            end
""")
    code=code.replace("    end\n    write_status('auditing');audit_margin_grid49_worker3d(studyDir,workerIndex);",
        """        if ~mapAborted
            ready=struct('seed',m.seed,'owner',owner,'auxiliary',workerIndex,'completeTasks',49);
            readyFile=fullfile(directory,sprintf('ready_%06d.json',m.seed));
            fid=fopen([readyFile '.tmp'],'w','n','UTF-8');fprintf(fid,'%s',jsonencode(ready));fclose(fid);
            movefile([readyFile '.tmp'],readyFile,'f');
        end
    end
""")
    assert code!=original and 'audit_margin_grid49_worker3d(studyDir,workerIndex)' not in code
    # These statements execute the numerical work; all must remain byte-identical.
    for needle in ['[route,info,preparedCache]=plan_layers3d',
                   'r=run_simulation3d(q,scene,preparedCache);',
                   'row=clearance_trial_record3d(job,q,scene,mapHash,r,errorText,toc(clockTrial));']:
        assert code.count(needle)==original.count(needle)==1
    (source/'run_margin_grid49_aux3d.m').write_text(code,encoding='utf-8')
    (source/'scheduling_only.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True),code.splitlines(True),
        fromfile='frozen_runner',tofile='auxiliary_runner')),encoding='utf-8')
    shutil.copy2(Path(__file__),source/Path(__file__).name)
    candidates=[]
    for row in m.read_csv(study/'map_manifest.csv'):
        seed=int(row['seed'])
        if seed>=2150 and m.yes(row['initialHardSafe']):
            owner=(seed-2001)%6+1
            if not any((study/'workers'/f'worker_{owner:02d}'/'trials').glob(f'*_s{seed:06d}.mat')):
                candidates.append(seed)
    candidates.sort(reverse=True);groups=[candidates[::2],candidates[1::2]]
    assert groups[0] and groups[1] and not set(groups[0])&set(groups[1])
    receipt=dict(originalOwnerGroups=6,auxiliaryProcesses=2,maxConcurrentProcesses=8,
        sourceCommit=m.git('rev-parse','HEAD'),groups=groups,addedAfterStart=True,
        reason='Use spare RAM/CPU on future maps; no changes to numerical functions or parameter grid.',
        publishingRule='Complete 49-task map checkpoints only; Windows no-replace rename; original owners audit canonical results.',
        sourceFiles=[dict(file=f.relative_to(study).as_posix(),sha256=m.digest(f))
                     for f in sorted(source.iterdir()) if f.is_file()])
    m.write_json(study/'aux_execution_protocol.json',receipt)
    print(json.dumps(receipt,ensure_ascii=False),flush=True);return receipt

def publish_ready(study,processed):
    receipts=[]
    for ready in (study/'auxiliary_runs').glob('aux_*/ready_*.json'):
        key=ready.as_posix()
        if key in processed:continue
        value=json.loads(ready.read_text(encoding='utf-8-sig'));seed=value['seed'];owner=value['owner']
        source=ready.parent/'pending_results';target=study/'workers'/f'worker_{owner:02d}'/'trials'
        files=sorted(source.glob(f'*_s{seed:06d}.mat'));assert len(files)==49
        target.mkdir(parents=True,exist_ok=True);results=[]
        for f in files:
            dst=target/f.name;temporary=target/(f.name+f'.aux_{value["auxiliary"]}.pending')
            # Copy first, then atomically publish without replacing an existing file.
            if dst.exists():results.append(dict(jobId=f.stem,published=False,reason='owner_already_saved'));continue
            shutil.copy2(f,temporary);assert m.digest(f)==m.digest(temporary)
            try:
                os.rename(temporary,dst)
                results.append(dict(jobId=f.stem,published=True,sha256=m.digest(dst)))
            except FileExistsError:
                results.append(dict(jobId=f.stem,published=False,reason='owner_saved_during_publish'))
        # Cold-plan evidence remains in the auxiliary folder, avoiding concurrent
        # writes to an owner's plan files. Each canonical MAT also has its route.
        receipt=dict(seed=seed,owner=owner,auxiliary=value['auxiliary'],results=results,
            published=sum(r['published'] for r in results))
        m.write_json(ready.parent/f'published_{seed:06d}.json',receipt)
        receipts.append(receipt);processed.add(key)
        print(f'AUX_MAP_PUBLISHED seed={seed} owner={owner} tasks={receipt["published"]}',flush=True)
    return receipts

def run(study):
    protocol=prepare(study);processes=[];logs=[];processed=set();start=time.monotonic();last=start
    for index,seeds in enumerate(protocol['groups'],1):
        vector=' '.join(map(str,seeds))
        code=f"restoredefaultpath;addpath('{(study/'aux_runtime_source').as_posix()}');run_margin_grid49_aux3d('{study.as_posix()}',{index},[{vector}]);"
        p,log=m.launch(study,f'aux_{index:02d}',code);processes.append(p);logs.append(log)
    try:
        while any(p.poll() is None for p in processes):
            time.sleep(2);publish_ready(study,processed)
            if time.monotonic()-last>=25:
                canonical=sum(1 for f in (study/'workers').glob('worker_*/trials/*.mat'))
                print(f'AUX_RUNNING elapsed={time.monotonic()-start:.0f}s canonicalRawFiles={canonical}/9800 publishedMaps={len(processed)}',flush=True)
                last=time.monotonic()
        publish_ready(study,processed)
        exits=[p.returncode for p in processes]
        assert all(v==0 for v in exits),exits
    finally:
        for log in logs:log.close()
    m.verify(study)
    m.write_json(study/'aux_execution_receipt.json',dict(passed=True,exitCodes=exits,publishedMaps=len(processed),
        wallSeconds=time.monotonic()-start,originalSourcesAndInputsUnchanged=True,
        note='Original-owner raw audits determine canonical validity; pending duplicates are retained outside the denominator.'))
    print('AUXILIARY_EXECUTION_COMPLETE',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--study',type=Path,required=True);args=p.parse_args();run(args.study)

"""Two more scheduling slots reuse the frozen auxiliary MATLAB runner.

The first auxiliary orchestrator remains the sole publisher until it finishes.
This process then takes over publication, retaining every earlier receipt.
"""
from __future__ import annotations
import argparse,ctypes,json,shutil,time
from ctypes import wintypes
from pathlib import Path
import margin_grid49 as m
from margin_grid49_aux import publish_ready

class MemoryStatus(ctypes.Structure):
    _fields_=[('length',wintypes.DWORD),('load',wintypes.DWORD)]+[
        (name,ctypes.c_ulonglong) for name in ['total','available','pageTotal','pageAvailable',
                                             'virtualTotal','virtualAvailable','extended']]

def run(study):
    m.verify(study)
    original=json.loads((study/'aux_execution_protocol.json').read_text(encoding='utf-8'))
    for item in original['sourceFiles']:
        assert m.digest(study/item['file'])==item['sha256']
    memory=MemoryStatus();memory.length=ctypes.sizeof(memory)
    assert ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(memory))
    assert memory.available>=4*2**30,'Less than 4 GiB free: do not add MATLAB processes.'
    reserved=set(sum(original['groups'],[]));groups=[]
    for owner in [3,5]:
        seeds=[]
        for row in m.read_csv(study/'map_manifest.csv'):
            seed=int(row['seed'])
            if 2070<=seed<2150 and (seed-2001)%6+1==owner and m.yes(row['initialHardSafe']):
                if seed not in reserved and not any((study/'workers'/f'worker_{owner:02d}'/'trials').glob(f'*_s{seed:06d}.mat')):
                    seeds.append(seed)
        groups.append(sorted(seeds,reverse=True))
    assert all(groups) and not set(groups[0])&set(groups[1])
    source=study/'more_aux_runtime_source';source.mkdir(exist_ok=True)
    shutil.copy2(Path(__file__),source/Path(__file__).name)
    protocol=dict(maxConcurrentProcesses=10,auxiliaryIndices=[3,4],ownerFocus=[3,5],groups=groups,
                  freeGiBAtLaunch=memory.available/2**30,sourceCommit=m.git('rev-parse','HEAD'),
                  matlabRunner='aux_runtime_source/run_margin_grid49_aux3d.m',
                  numericalCoreUnchanged=True,publishingRule='Original auxiliary publisher first, exclusive handoff after its completion receipt.',
                  sourceFiles=[dict(file=f.relative_to(study).as_posix(),sha256=m.digest(f)) for f in sorted(source.iterdir()) if f.is_file()])
    m.write_json(study/'more_aux_execution_protocol.json',protocol)
    print(json.dumps(protocol,ensure_ascii=False),flush=True)
    processes=[];logs=[];start=time.monotonic();last=start;processed=set();publisher=False
    for index,seeds in zip([3,4],groups):
        vector=' '.join(map(str,seeds))
        code=f"restoredefaultpath;addpath('{(study/'aux_runtime_source').as_posix()}');run_margin_grid49_aux3d('{study.as_posix()}',{index},[{vector}]);"
        process,log=m.launch(study,f'aux_{index:02d}',code);processes.append(process);logs.append(log)
    try:
        while any(p.poll() is None for p in processes) or not (study/'aux_execution_receipt.json').is_file():
            time.sleep(2)
            if (study/'aux_execution_receipt.json').is_file():
                if not publisher:
                    receipt=json.loads((study/'aux_execution_receipt.json').read_text(encoding='utf-8'))
                    assert receipt['passed']
                    for path in (study/'auxiliary_runs').glob('aux_*/published_*.json'):
                        processed.add((path.parent/path.name.replace('published_','ready_')).as_posix())
                    publisher=True
                publish_ready(study,processed)
            if time.monotonic()-last>=25:
                canonical=sum(1 for _ in (study/'workers').glob('worker_*/trials/*.mat'))
                print(f'MORE_AUX_RUNNING elapsed={time.monotonic()-start:.0f}s canonicalRawFiles={canonical}/9800 publisher={publisher}',flush=True)
                last=time.monotonic()
        for path in (study/'auxiliary_runs').glob('aux_*/published_*.json'):
            processed.add((path.parent/path.name.replace('published_','ready_')).as_posix())
        publish_ready(study,processed)
        exits=[p.returncode for p in processes];assert all(code==0 for code in exits),exits
    finally:
        for log in logs:log.close()
    m.verify(study)
    for item in protocol['sourceFiles']:assert m.digest(study/item['file'])==item['sha256']
    m.write_json(study/'more_aux_execution_receipt.json',dict(passed=True,exitCodes=exits,
        publishedMaps=len(processed),wallSeconds=time.monotonic()-start,
        originalSourcesAndInputsUnchanged=True,maxConcurrentProcesses=10))
    print('MORE_AUXILIARY_EXECUTION_COMPLETE',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--study',type=Path,required=True);args=p.parse_args();run(args.study)

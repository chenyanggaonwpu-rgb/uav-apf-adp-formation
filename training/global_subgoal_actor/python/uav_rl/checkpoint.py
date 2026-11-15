from pathlib import Path
import os,time
import torch
from .io import atomic_json,sha256,read_json

JOURNALS=['flights.jsonl','losses.jsonl','metrics.jsonl','events.jsonl']


def save(run,learner,replay,state,config,kind='latest'):
    run=Path(run);folder=run/'checkpoints';folder.mkdir(exist_ok=True)
    serial=int(state.get('checkpoint_serial',0))+1;state['checkpoint_serial']=serial
    path=folder/f'c{serial:07d}_u{learner.updates:09d}.pt';assert not path.exists()
    offsets={name:(run/name).stat().st_size if (run/name).exists() else 0 for name in JOURNALS}
    value={'schema':1,'config_sha256':config['_sha256'],'source_sha256':state['source_sha256'],'learner':learner.state_dict(),
        'replay':replay.state_dict(),'state':dict(state),'journal_offsets':offsets}
    temporary=path.with_name(path.name+'.part');torch.save(value,temporary);os.replace(temporary,path)
    receipt={'path':str(path),'sha256':sha256(path),'kind':kind,'updates':learner.updates,'actor_updates':learner.actor_updates,'bc_updates':0,
        'flights':state['flights'],'config_sha256':config['_sha256']}
    atomic_json(run/'latest_checkpoint.json',receipt)
    return receipt


def load(receipt,run,learner,replay,config,reconcile=True):
    assert sha256(receipt['path'])==receipt['sha256'],'Checkpoint hash mismatch.'
    saved=torch.load(receipt['path'],map_location='cpu',weights_only=True)
    assert saved['schema']==1 and saved['config_sha256']==config['_sha256'],'Use a new run for changed settings.'
    learner.load_state_dict(saved['learner']);replay.load_state_dict(saved['replay'],Path(run)/'episodes')
    if reconcile:
        for name,offset in saved['journal_offsets'].items():
            path=Path(run)/name
            if path.exists() and path.stat().st_size>offset:
                recovery=Path(run)/'recovery';recovery.mkdir(exist_ok=True)
                with path.open('rb') as source:
                    source.seek(offset);tail=source.read()
                (recovery/f'{time.time_ns()}_{name}').write_bytes(tail)
                with path.open('r+b') as stream:stream.truncate(offset)
    return saved['state']


def trim(run,keep=8):
    run=Path(run);pinned=set()
    for name in ['latest_checkpoint.json','warm_anchor_checkpoint.json','best_model.json']:
        path=run/name
        if path.exists():
            item=read_json(path);value=item.get('checkpoint',item)
            if isinstance(value,dict) and value.get('path'):pinned.add(str(Path(value['path']).resolve()))
    for path in (run/'evaluations').glob('*/selection.json'):
        selection=read_json(path)
        if selection.get('level')!='full':continue
        value=selection.get('checkpoint',{})
        if value.get('path'):pinned.add(str(Path(value['path']).resolve()))
    files=sorted((run/'checkpoints').glob('*.pt'),key=lambda p:p.name)
    for path in files[:-keep]:
        if str(path.resolve()) not in pinned:path.unlink()

"""Rebind frozen geometry/weather to the uniform full-mission planning task."""
from pathlib import Path
import copy,collections
from .config import FAMILIES
from .maps import physical_digest
from .io import ROOT,atomic_json,read_json,digest,stamp,sha256

PROTOCOL='global_subgoal_v1'

def recipe_digest(cfg):
    return digest({'protocol':PROTOCOL,'dataset':cfg['dataset'],'environment':cfg['environment'],
        'source_reservation_sha256':sha256(ROOT/'datasets/frozen/reservation.json')})

def bind_case(original):
    c=copy.deepcopy(original)
    c['previous_case_sha256']=c.pop('case_sha256',None)
    c['previous_context']=c.get('context')
    c.update(schema=6,context='global_planning',task_protocol=PROTOCOL)
    assert physical_digest(c)==c['geometry_sha256']
    c['case_sha256']=digest(c)
    return c

def prepare(cfg,workroot):
    locked=cfg.get('continuation_dataset_binding')
    if locked:
        folder=Path(locked['folder']);folder=folder if folder.is_absolute() else ROOT/folder;reservation=read_json(folder/'reservation.json')
        for split in ['validation','test','micro']:
            expected=locked[split+'_digest']
            assert reservation[split+'_digest']==expected and digest(read_json(folder/(split+'.json')))==expected
        return folder
    recipe=recipe_digest(cfg);destination=Path(workroot)/'datasets'/recipe[:12]
    destination.mkdir(parents=True,exist_ok=True)
    source=ROOT/'datasets/frozen';converted={}
    for split,total in [('validation',256),('test',512),('micro',32)]:
        original=read_json(source/f'{split}.json');assert len(original)==total
        cases=[bind_case(c) for c in original];converted[split]=cases
        path=destination/f'{split}.json'
        if path.exists():assert digest(read_json(path))==digest(cases)
        else:atomic_json(path,cases)
        if split!='micro':
            geometry_receipt=read_json(source/f'{split}_native_validation.json')
            assert geometry_receipt['passed'] and geometry_receipt['maps']==total
            receipt={'passed':True,'maps':total,'geometry_unchanged':True,
                'source_receipt_sha256':sha256(source/f'{split}_native_validation.json'),
                'scope':'Reuse geometry/initial-braking/continuous-witness certificate only; no old controller outcomes reused.'}
            atomic_json(destination/f'{split}_geometry_validation.json',receipt)
    v,t,m=(converted[s] for s in ['validation','test','micro'])
    assert not {c['geometry_sha256'] for c in v}&{c['geometry_sha256'] for c in t}
    assert len({c['geometry_sha256'] for c in v+t})==768
    assert {c['id'] for c in m}<={c['id'] for c in v}
    reservation={'prepared_at':stamp(),'recipe_sha256':recipe,'protocol':PROTOCOL,
        'validation_maps':256,'test_maps':512,'validation_digest':digest(v),'test_digest':digest(t),'micro_digest':digest(m),
        'source_reservation_sha256':sha256(source/'reservation.json'),'geometry_disjoint':True,
        'weather_and_geometry_preserved':True,'all_arms_same_context':True,'stale_waypoint_fault':False,
        'test_used_for_selection':False,'teacher_used':False,'bamboo_route_carving':False,'families':FAMILIES,
        'context_counts':dict(collections.Counter(c['context'] for c in v+t))}
    path=destination/'reservation.json'
    if path.exists():
        old=read_json(path)
        for k in ['recipe_sha256','validation_digest','test_digest','micro_digest']:assert old[k]==reservation[k]
    else:atomic_json(path,reservation)
    return destination

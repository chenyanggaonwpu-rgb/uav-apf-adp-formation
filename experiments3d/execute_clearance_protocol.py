"""Run remaining frozen numerical stages sequentially, with four workers maximum."""
from __future__ import annotations
import argparse, hashlib, importlib.util, json, time
from pathlib import Path

STAGES=['screen','refine','nominal','robust','select','holdout','boundary','geometry','ablation','timing']

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('study',type=Path); parser.add_argument('--start',choices=STAGES,default='screen')
    parser.add_argument('--wait-for-stage',choices=STAGES)
    parser.add_argument('--workers',type=int,default=4); args=parser.parse_args()
    study=args.study.resolve(); root=Path(__file__).resolve().parents[1]
    frozen=study/'source_snapshot/experiments3d/clearance_study.py'
    spec=importlib.util.spec_from_file_location('frozen_protocol_controller',frozen)
    controller=importlib.util.module_from_spec(spec); spec.loader.exec_module(controller)
    # Only log/runtime working directories are redirected; numerical execution
    # still uses the immutable MATLAB functions below source_snapshot.
    controller.ROOT=root
    started=time.monotonic(); freeze=study/'selection_freeze.json'
    if args.wait_for_stage:
        print('WAITING_FOR_STAGE '+args.wait_for_stage,flush=True)
        while not (study/f'{args.wait_for_stage}_summary.json').exists(): time.sleep(2)
        controller.collect(study,args.wait_for_stage)
    for stage in STAGES[STAGES.index(args.start):]:
        print('PROTOCOL_STAGE_BEGIN '+stage,flush=True)
        if stage in ('holdout','boundary','geometry','ablation','timing'):
            frozen_selection=json.loads(freeze.read_text(encoding='utf-8'))
            assert controller.sha256(study/'selection.json')==frozen_selection['sha256'], 'Final candidate was changed after freeze'
        jobs=controller.create_stage(study,stage)
        if jobs is not None: controller.run_workers(study,jobs,1 if stage=='timing' else args.workers)
        if stage=='select':
            entry=dict(sha256=controller.sha256(study/'selection.json'),createdLocal=time.strftime('%Y-%m-%d %H:%M:%S'),
                       holdoutResultsExist=(study/'holdout_results.csv').exists(),rule='No candidate change after freeze')
            assert not entry['holdoutResultsExist']
            controller.write_json(freeze,entry)
        print('PROTOCOL_STAGE_COMPLETE '+stage,flush=True)
    controller.write_json(study/'execution_complete.json',dict(stages=STAGES,wallSeconds=time.monotonic()-started,
                          startStage=args.start,workers=args.workers,completedLocal=time.strftime('%Y-%m-%d %H:%M:%S')))
    print('ALL_NUMERICAL_STAGES_COMPLETE',flush=True)

"""Compare deterministic legacy replay on exactly the same frozen maps."""
import argparse, csv, json, math
from pathlib import Path

def read(path):
    with path.open(encoding='utf-8-sig',newline='') as f: return list(csv.DictReader(f))
if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('study',type=Path); args=parser.parse_args(); study=args.study.resolve()
    old={int(r['seed']):r for r in read(study/'baseline_trials.csv')}
    fields=['time','path','formationRms','minObstacleClearance','minPairDistance','verticalTravel','finalLeaderError','finalFormationError','finalMaxSpeed']
    rows=[]
    for file in (study/'stages/screen').glob('worker_*/trials.csv'): rows+=read(file)
    replay=[r for r in rows if r['configId']=='baseline']; assert len(replay)==8
    maxima={f:0 for f in fields}
    for row in replay:
        original=old[int(row['seed'])]; assert row['reason']==original['reason'] and row['success']==original['success']
        for field in fields:
            a,b=float(row[field]),float(original[field])
            if math.isnan(a) and math.isnan(b): continue
            assert math.isfinite(a) and math.isfinite(b)
            maxima[field]=max(maxima[field],abs(a-b)); assert abs(a-b)<=1e-8, (row['seed'],field,a,b)
    result=dict(passed=True,records=len(replay),maxAbsoluteDifferences=maxima,tolerance=1e-8,
                rule='Exact same original map seed and legacy mode, stochastic repeat zero; no timing equality claim')
    (study/'baseline_replay_check.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result),flush=True)

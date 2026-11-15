"""Commit all model SHA values before the first TEST512 evaluation."""
from pathlib import Path
import argparse,sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from uav_rl.io import read_json,atomic_json,sha256,stamp
def main():
    p=argparse.ArgumentParser();p.add_argument('--models',nargs='+',required=True);p.add_argument('--primary',required=True);p.add_argument('--work-root',default='F:/UAV_Schema6');args=p.parse_args()
    files=[Path(x).resolve() for x in args.models];primary=Path(args.primary).resolve();assert primary in files
    panel={'locked_at':stamp(),'models':[{'path':str(f),'sha256':sha256(f)} for f in files],'primary_sha256':sha256(primary),
        'test_digest':read_json(ROOT/'datasets/frozen/reservation.json')['test_digest'],'test_used_for_selection':False}
    work=Path(args.work_root).resolve();assert work.drive.upper()!='C:';target=work/'FINAL_MODEL_PANEL.json';assert not target.exists(),'Model panel is immutable.'
    atomic_json(target,panel);print(target)
if __name__=='__main__':main()

"""Download a release asset, verify its SHA, and extract without path traversal."""
from pathlib import Path
import argparse,sys,urllib.request,hashlib,zipfile,os
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from uav_rl.io import read_json,sha256,atomic_json
def main():
    p=argparse.ArgumentParser();p.add_argument('--work-root',default='F:/UAV_Schema6');p.add_argument('--archive');args=p.parse_args()
    work=Path(args.work_root).resolve();assert work.drive.upper()!='C:';work.mkdir(parents=True,exist_ok=True)
    entry=read_json(ROOT/'assets/CHECKPOINT_DOWNLOAD.json');archive=Path(args.archive).resolve() if args.archive else work/'downloads'/entry['file'];archive.parent.mkdir(parents=True,exist_ok=True)
    if not archive.exists():
        part=archive.with_name(archive.name+'.part');print('Downloading',entry['download_url'],flush=True)
        req=urllib.request.Request(entry['download_url'],headers={'User-Agent':'schema6-handoff'})
        with urllib.request.urlopen(req,timeout=1800) as src,part.open('wb') as target:
            for block in iter(lambda:src.read(1024*1024),b''):target.write(block)
        os.replace(part,archive)
    assert archive.stat().st_size==entry['bytes'] and sha256(archive)==entry['sha256'],'Download checksum mismatch.'
    destination=work/'checkpoint_assets'
    with zipfile.ZipFile(archive) as z:
        for item in z.infolist():assert (destination/item.filename).resolve().is_relative_to(destination.resolve()),'Unsafe archive member.'
        assert z.testzip() is None;z.extractall(destination)
    meta=read_json(destination/'CHECKPOINT_ASSETS.json')
    for name,m in meta.items():
        assert sha256(destination/m['checkpoint_file'])==m['checkpoint_sha256']
        assert sha256(destination/m['actor_file'])==m['actor_sha256']
        for ep in m['replay_episodes']:assert sha256(destination/ep['file'])==ep['sha256']
    atomic_json(destination/'EXTRACTED_AND_VERIFIED.json',{'passed':True,'archive_sha256':entry['sha256'],'assets':list(meta)})
    print('CHECKPOINT_ASSETS_VERIFIED',destination,flush=True)
if __name__=='__main__':main()

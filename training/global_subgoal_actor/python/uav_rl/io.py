from pathlib import Path
import hashlib,json,os,time

ROOT=Path(__file__).resolve().parents[2]


def sha256(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def atomic_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);temporary=path.with_name(path.name+'.part')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    for attempt in range(100):
        try:os.replace(temporary,path);return
        except PermissionError:
            if attempt==99:raise
            time.sleep(.05)


def read_json(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def append_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('a',encoding='utf-8',newline='\n') as stream:
        stream.write(json.dumps(value,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n');stream.flush()


def stamp():return time.strftime('%Y-%m-%d %H:%M:%S')


def digest(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def source_manifest():
    return {p.relative_to(ROOT).as_posix():sha256(p) for folder in ['python','config','matlab']
            for p in sorted((ROOT/folder).rglob('*')) if p.is_file() and p.suffix in ['.py','.m','.json'] and '__pycache__' not in p.parts}

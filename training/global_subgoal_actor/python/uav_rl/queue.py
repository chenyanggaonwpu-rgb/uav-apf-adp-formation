from pathlib import Path
import hashlib,os,subprocess,time
from .io import ROOT,atomic_json,read_json,sha256


def worker_environment(work):
    work=Path(work)
    for name in ['cache','temp','preferences']:(work/name).mkdir(parents=True,exist_ok=True)
    env=os.environ.copy();env.update(MCR_CACHE_ROOT=str(work/'cache'),TEMP=str(work/'temp'),TMP=str(work/'temp'),MATLAB_PREFDIR=str(work/'preferences'),
        OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    runtime=env.get('MATLAB_RUNTIME_ROOT')
    if runtime:
        env['PATH']=os.pathsep.join([str(Path(runtime)/name/'win64') for name in ['runtime','bin','sys/os']])+os.pathsep+env.get('PATH','')
    return env


class Queue:
    def __init__(self,root,workroot,cfg):
        self.root=Path(root).resolve();self.workroot=Path(workroot).resolve();self.cfg=cfg;self.workers=[];self.wait_started={}
        self.activity_signature=None;self.last_activity=time.monotonic()
        for name in ['queued','claimed','completed','heartbeat','logs']:(self.root/name).mkdir(parents=True,exist_ok=True)
    @property
    def executable(self):return ROOT/'worker_windows/uav_rl_worker.exe'
    def result(self,key):return self.root/'completed'/f'{key}.mat'
    def ready(self,key):return self.result(key).exists() and Path(str(self.result(key))+'.json').exists()
    def submit(self,job):
        key=job['id'];assert key and all(c.isalnum() or c in '_-' for c in key)
        if self.ready(key) or any((self.root/name/f'{key}.json').exists() for name in ['queued','claimed']):return
        atomic_json(self.root/'queued'/f'{key}.json',job)
    def stop_orphans(self):
        # Receipts name only workers started for this exact queue. Verify
        # command lines before terminating; never touch other MATLAB jobs.
        import psutil
        receipt=self.root/'worker_pids.json'
        if not receipt.exists():return
        for pid in read_json(receipt)['pids']:
            try:
                process=psutil.Process(pid);cmd=process.cmdline()
                if str(self.root) in cmd and any(Path(arg).name.lower()=='uav_rl_worker.exe' for arg in cmd):
                    process.terminate()
                    try:process.wait(15)
                    except psutil.TimeoutExpired:process.kill();process.wait(10)
            except (psutil.NoSuchProcess,psutil.ZombieProcess):pass
    def recover(self):
        assert not self.workers
        self.stop_orphans()
        for file in (self.root/'claimed').glob('*.json'):
            if not self.ready(file.stem):os.replace(file,self.root/'queued'/file.name)
            else:file.unlink()
        for lock in (self.root/'claimed').glob('*.json.lock'):lock.rmdir()
    def start(self,count=None):
        assert not self.workers
        self.wait_started={};self.activity_signature=None;self.last_activity=time.monotonic()
        stop=self.root/'STOP'
        if stop.exists():stop.unlink()
        identity=hashlib.sha256(str(self.root).encode()).hexdigest()[:10]
        locations=[];atomic_json(self.root/'worker_pids.json',{'pids':[],'executable_sha256':sha256(self.executable),'runtime_paths':[]})
        for i in range(count or self.cfg['system']['workers']):
            work=self.workroot/'mcr'/identity/f'w{i}';work.mkdir(parents=True,exist_ok=True)
            with (self.root/'logs'/f'w{i}.txt').open('ab') as log:
                p=subprocess.Popen([str(self.executable),'queue',str(self.root),f'w{i}',str(ROOT/'assets')],cwd=work,env=worker_environment(work),
                    stdout=log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            self.workers.append(p);locations.append(str(work))
            atomic_json(self.root/'worker_pids.json',{'pids':[p.pid for p in self.workers],'executable_sha256':sha256(self.executable),'runtime_paths':locations})
    def check(self,jobs):
        completed=0;active=[]
        for job in jobs:
            error=Path(str(self.result(job['id']))+'.error.json')
            if error.exists():raise RuntimeError(error.read_text(encoding='utf-8'))
            if self.ready(job['id']):
                completed+=1;self.wait_started.pop(job['id'],None)
            elif (self.root/'claimed'/f'{job["id"]}.json').exists():
                # Queue waiting time is not native execution time. A full
                # validation contains many more tasks than worker slots.
                active.append(job['id'])
                started=self.wait_started.setdefault(job['id'],time.monotonic())
                if time.monotonic()-started>self.cfg['system']['worker_timeout_seconds']:raise TimeoutError(f'Native job {job["id"]} exceeded its watchdog budget; checkpoint recovery retains its identity.')
            else:self.wait_started.pop(job['id'],None)
        signature=(completed,tuple(active))
        if signature!=self.activity_signature:self.activity_signature=signature;self.last_activity=time.monotonic()
        if completed<len(jobs) and not active and time.monotonic()-self.last_activity>self.cfg['system']['worker_timeout_seconds']:
            raise TimeoutError('Native queue did not claim any pending job within its startup/idle watchdog budget.')
        for p in self.workers:
            if p.poll() is not None:raise RuntimeError(f'Native worker {p.pid} exited: {p.returncode}. See {self.root / "logs"}.')
    def stop(self,force=False):
        (self.root/'STOP').touch()
        for p in self.workers:
            try:p.wait(15 if not force else 2)
            except subprocess.TimeoutExpired:
                if not force:raise RuntimeError('Drain in-flight jobs before stopping workers, or explicitly force a checkpoint recovery.')
                p.terminate();p.wait(15)
        self.workers=[]


def make_job(case,arm,identifier,model=None,stochastic=False,seed=1,environment=None):
    mode='none' if arm in ['apf','global_rule'] else 'rule' if arm=='rule' else 'actor'
    if environment is None:
        from .config import load_config
        environment=load_config()['environment']
    job={'planner':dict(environment),'id':identifier,'scene':case,'arm':arm,'mode':mode,'stochastic':bool(stochastic),'teacher_probability':0,'policy_seed':int(seed),'task_protocol':'global_subgoal_v1'}
    if mode=='actor':job.update(model=str(Path(model).resolve()),model_sha256=sha256(model))
    return job


def native_command(action,first,second,workroot,timeout=1200):
    work=Path(workroot)/'native_checks';work.mkdir(parents=True,exist_ok=True)
    with (work/f'{action}.log').open('ab') as log:
        process=subprocess.run([str(ROOT/'worker_windows/uav_rl_worker.exe'),action,str(first),str(second),str(ROOT/'assets')],
            cwd=work,env=worker_environment(work),stdout=log,stderr=subprocess.STDOUT,timeout=timeout,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    if process.returncode:raise RuntimeError(f'Native {action} exited {process.returncode}; inspect {work / (action+".log")}. MATLAB Runtime must be R2024a x64.')
    return read_json(second if action in ['validate','encode'] else first)

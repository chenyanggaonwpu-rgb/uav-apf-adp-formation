from pathlib import Path
import json,time,webbrowser
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlparse
from .io import read_json

PAGE='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>UAV 全局子目标规划 CPU 强化学习进度</title><style>
:root{font-family:system-ui,"Microsoft YaHei",sans-serif;color:#203441;background:#f1f4f5}body{max-width:1160px;margin:34px auto;padding:0 22px}header{display:flex;justify-content:space-between;align-items:baseline}h1{font-size:25px;margin:0}p{color:#5d707c}#phase{font-size:18px;color:#166c68}.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:14px}.card,.panel{background:white;border:1px solid #dbe3e5;border-radius:9px;padding:18px}.card strong{font-size:27px;display:block;margin-top:8px}.card span{font-size:13px;color:#677984}progress{width:100%;height:16px;accent-color:#267e76}.plots{display:grid;grid-template-columns:1fr 1fr;gap:16px;margin:18px 0}svg{width:100%;height:180px;overflow:visible}.panel h2{font-size:16px;margin-top:0}pre{white-space:pre-wrap;word-break:break-all;font-size:12px}.small{font-size:12px;color:#677984}#error{color:#a23d39}button{background:#286f6b;color:white;border:0;border-radius:5px;padding:8px 12px}footer{margin:20px 0;color:#677984;font-size:12px}@media(max-width:760px){.cards{grid-template-columns:1fr 1fr}.plots{grid-template-columns:1fr}header{display:block}}
</style><header><h1>五机全局子目标规划 · CPU 强化学习</h1><span id="time" class="small">连接中</span></header><p id="phase">读取训练进度…</p>
<div class="cards"><div class="card"><span>真实五机任务 / 预算</span><strong id="flights">—</strong></div><div class="card"><span>实际 Actor 更新 / Critic 更新</span><strong id="updates">—</strong></div><div class="card"><span>有效动作转移 / 独立地图</span><strong id="data">—</strong></div><div class="card"><span>空闲 RAM / Replay 内存</span><strong id="memory">—</strong></div></div>
<p><progress id="bar" value="0" max="1"></progress></p><p class="small" id="detail"></p><p id="error"></p>
<div class="validation-cards" style="display:grid;grid-template-columns:1fr 1fr;gap:16px;margin:18px 0"><section class="panel"><h2>最新小验证 · 32图</h2><strong id="micro_result" style="font-size:25px">等待本阶段评估</strong><p id="micro_info" class="small"></p></section><section class="panel"><h2>最新完整验证 · 256图</h2><strong id="full_result" style="font-size:25px">等待本阶段评估</strong><p id="full_info" class="small"></p></section></div><div class="plots"><section class="panel"><h2>固定验证：安全完成五机任务</h2><svg id="success" viewBox="0 0 500 180"></svg><div class="small">绿色：32图小验证；橙色：256图完整验证。各自按评估轮次绘制，失败候选会回退。</div></section><section class="panel"><h2>Critic 损失 / Actor 损失</h2><svg id="loss" viewBox="0 0 500 180"></svg><div class="small">损失下降不等于闭环性能提升；以左侧独立验证为准。</div></section></div>
<section class="panel"><h2>当前最优验证版本</h2><pre id="best">等待完整验证</pre></section><details class="panel" style="margin-top:16px"><summary>本次参数与训练配置</summary><pre id="config"></pre></details>
<footer>本页面只读，每 2 秒刷新。本机 CPU 同时负责梯度计算与 MATLAB Runtime 仿真；双 worker、2个Torch线程、batch32，所有记录在 F 盘。BC 更新及专家查询必须始终为 0。</footer>
<script>
const $=id=>document.getElementById(id);const phases={waiting_for_free_memory:'等待释放内存 · 未关闭其他应用',collecting_real_native_rollouts:'采集真实原生仿真轨迹',critic_warmup_actor_frozen:'预热 Critic · Actor 保持冻结',reinforcement_learning:'强化学习迭代',validation_micro:'固定小验证集评估',validation_full:'固定完整验证集评估',paused:'已安全暂停',needs_attention:'已暂停，需要检查',training_budget_complete_ready_for_independent_test:'训练预算完成，等待独立测试与冻结'};
function plot(id,series){const svg=$(id);while(svg.firstChild)svg.removeChild(svg.firstChild);let all=series.flatMap(s=>s.values).filter(Number.isFinite);if(!all.length)return;let lo=Math.min(0,...all),hi=Math.max(1,...all);series.forEach(s=>{let p=s.values.map((v,i)=>`${15+i*470/Math.max(1,s.values.length-1)},${160-(v-lo)*140/(hi-lo)}`).join(' ');let n=document.createElementNS('http://www.w3.org/2000/svg','polyline');n.setAttribute('points',p);n.setAttribute('fill','none');n.setAttribute('stroke',s.color);n.setAttribute('stroke-width','2');svg.appendChild(n)});let text=document.createElementNS('http://www.w3.org/2000/svg','text');text.setAttribute('x','15');text.setAttribute('y','13');text.setAttribute('font-size','11');text.textContent=`范围 ${lo.toFixed(2)} ～ ${hi.toFixed(2)}`;svg.appendChild(text)}
function validationCards(d){for(const level of ['micro','full']){const values=(d.metrics||[]).filter(x=>x.level===level);const latest=values.at(-1);const result=$(level+'_result'),info=$(level+'_info');if(latest){const c=latest.gate.candidate;result.textContent=`${(100*c.safe_success/c.tasks).toFixed(1)}% (${c.safe_success}/${c.tasks})`;info.textContent=`${latest.timestamp} · 安全违规 ${c.unsafe} · ${latest.gate.admissible?'通过回退检查':'触发回退'}`;}else{result.textContent='等待本阶段评估';const b=(d.status.reward_change_baseline||{})[level];info.textContent=b?`奖励调整前：${(100*b.safe_success/b.tasks).toFixed(1)}% (${b.safe_success}/${b.tasks}) · ${b.timestamp}`:'本阶段尚无该类验证';}}}
async function update(){try{let d=await(await fetch('/api/status',{cache:'no-store'})).json();let s=d.status||{};validationCards(d);$('time').textContent=s.timestamp||'等待进度文件';$('phase').textContent=phases[s.phase]||s.phase||'等待训练开始';$('flights').textContent=`${s.flights||0} / ${s.maximum_flights||'—'}`;$('updates').textContent=`${s.actor_updates_in_lineage||0} / ${s.critic_updates_in_lineage||0}`;$('data').textContent=`${s.replay_transitions||0} / ${s.unique_maps_flown||0}`;$('memory').textContent=`${((s.free_ram_bytes||0)/1073741824).toFixed(2)} / ${((s.replay_bytes||0)/1073741824).toFixed(2)} GiB`;$('bar').max=s.maximum_flights||1;$('bar').value=s.flights||0;$('detail').textContent=`课程: ${s.campaign_stage_name||s.campaign_stage||0} · 截止: ${s.campaign_deadline||'—'} · 训练/探针 episode: ${s.replay_training_episodes||0}/${s.replay_probe_episodes||0} · 子目标周期: ${s.planner_decision_seconds||1.6}s · BC: ${s.bc_updates||0} · 专家查询: ${s.teacher_queries||0} · Critic 门槛: ${s.critic_ready?'已通过':'等待'} · 回退次数: ${s.validation_rollbacks||0} · ${s.evaluation?`评估 ${s.completed}/${s.total}`:''}`;$('error').textContent=s.error||'';$('best').textContent=s.best_model?JSON.stringify(s.best_model,null,2):'尚无完成全量验证的候选；不会将训练损失最低的模型自动当作最优。';$('config').textContent=JSON.stringify(d.config||{},null,2);plot('success',[{values:(d.metrics||[]).filter(x=>x.level==='micro').map(x=>100*x.gate.candidate.safe_success/x.gate.candidate.tasks),color:'#267e76'},{values:(d.metrics||[]).filter(x=>x.level==='full').map(x=>100*x.gate.candidate.safe_success/x.gate.candidate.tasks),color:'#d49443'}]);plot('loss',[{values:(d.losses||[]).map(x=>x.critic_loss),color:'#267e76'},{values:(d.losses||[]).map(x=>x.actor_loss).filter(Number.isFinite),color:'#d49443'}]);}catch(e){$('time').textContent='暂时读取失败，将自动重试'}}update();setInterval(update,2000);
</script></html>'''


def tail(path,limit=400):
    path=Path(path)
    if not path.exists():return []
    with path.open('rb') as stream:
        stream.seek(max(0,path.stat().st_size-2_000_000));data=stream.read().decode('utf-8',errors='replace').splitlines()
    result=[]
    for line in data[-limit:]:
        try:result.append(json.loads(line))
        except json.JSONDecodeError:pass
    return result


def snapshot(run):
    run=Path(run)
    def read(name):
        try:return read_json(run/name)
        except (OSError,ValueError):return {}
    return {'status':read('progress.json'),'config':read('configuration.json'),'metrics':tail(run/'metrics.jsonl',200),'losses':tail(run/'losses.jsonl',400)}


def serve(run,port=8765,open_browser=False):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            route=urlparse(self.path).path
            if route=='/':body=PAGE.encode('utf-8');content='text/html; charset=utf-8'
            elif route=='/api/status':body=json.dumps(snapshot(run),ensure_ascii=False,allow_nan=False).encode('utf-8');content='application/json; charset=utf-8'
            else:self.send_error(404);return
            self.send_response(200);self.send_header('Content-Type',content);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
        def log_message(self,*args):pass
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler);url=f'http://127.0.0.1:{port}/';print(url,flush=True)
    if open_browser:webbrowser.open(url)
    try:server.serve_forever(poll_interval=.5)
    except KeyboardInterrupt:pass
    finally:server.server_close()


def watch(run):
    try:
        while True:
            s=snapshot(run)['status'];print(json.dumps({k:s.get(k) for k in ['timestamp','phase','flights','actor_updates_in_lineage','critic_updates_in_lineage','replay_transitions','bc_updates','critic_ready']},ensure_ascii=False),flush=True);time.sleep(5)
    except KeyboardInterrupt:return

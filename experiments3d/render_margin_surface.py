"""Offline, rotatable scientific surface viewer from audited grid CSV only."""
from __future__ import annotations
import csv,hashlib,json,sys
from pathlib import Path

HTML=r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>余量与成功率 · 49 组合</title><style>
:root{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI","Microsoft YaHei",sans-serif;color:#18222e;background:#fafbfd;font-synthesis:none}
*{box-sizing:border-box}body{margin:0}main{max-width:1440px;margin:0 auto;padding:36px 40px 24px}
.eyebrow{font-size:12px;letter-spacing:2px;color:#6b7785}h1{font-size:40px;letter-spacing:-1px;font-weight:600;margin:10px 0 12px}
.intro,.note{color:#687483;line-height:1.7;font-size:14px}.toolbar{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin:23px 0 5px}
button{border:1px solid #d5dce6;border-radius:8px;background:#fff;color:#273343;padding:9px 15px;font:inherit;font-size:13px;cursor:pointer}
button[aria-pressed=true]{background:#1766c4;color:#fff;border-color:#1766c4}button:focus-visible,a:focus-visible{outline:3px solid #e49e37;outline-offset:3px}
.layout{display:grid;grid-template-columns:minmax(0,1fr) 290px;gap:24px;align-items:start}.stage{position:relative;background:#fff;border-radius:12px}
canvas{display:block;width:100%;height:620px;touch-action:none;cursor:grab}.stage:active canvas{cursor:grabbing}
.aside{padding-top:24px}h2{font-size:17px;font-weight:600;margin:0 0 18px}.number{font-size:37px;font-weight:600;letter-spacing:-1px;margin:7px 0}
.hint{font-size:12px;color:#758292;line-height:1.6}.details{border-top:1px solid #e0e5ed;margin-top:24px;padding-top:20px}
dl{display:grid;grid-template-columns:1fr auto;font-size:13px;gap:10px;margin:14px 0}dt{color:#707d8b}dd{margin:0;font-variant-numeric:tabular-nums}
.legend{display:flex;gap:18px;flex-wrap:wrap;font-size:12px;color:#657383;padding:12px 18px}.dot{display:inline-block;width:9px;height:9px;background:#174578;border-radius:50%;margin-right:6px}.dot.base{background:#e98821}.dot.peak{background:#0d8c59}
a{color:#1766c4;text-decoration:none}.sources{margin-top:26px}.tablewrap{overflow:auto;margin-top:16px;max-height:440px}table{border-collapse:collapse;width:100%;font-size:13px;text-align:right}
th,td{padding:11px 13px;border-bottom:1px solid #e2e6ed;white-space:nowrap}th{background:#f0f4f9;position:sticky;top:0;color:#576477;font-weight:500}td:first-child,th:first-child{text-align:left}
.footer{display:flex;gap:22px;flex-wrap:wrap;border-top:1px solid #e1e6ef;margin-top:30px;padding-top:17px;font-size:12px}
@media(max-width:850px){main{padding:24px 18px}h1{font-size:29px}.layout{grid-template-columns:1fr}.aside{padding-top:0}.number{font-size:30px}canvas{height:460px}.toolbar{gap:7px}}
</style><main><div class="eyebrow">UAV FORMATION / PARAMETER STUDY</div><h1>余量与成功率</h1>
<div class="intro">49 个参数组合 × 同一批 200 张冻结地图。拖动旋转，滚轮缩放；悬停采样点查看精确结果。</div>
<div class="toolbar"><button id="all" aria-pressed="true">全部地图 · 200</button><button id="safe" aria-pressed="false">初始安全子集 · 157</button><button id="peak">查看本轮最高</button><button id="baseline">查看原基线</button><button id="reset">重置视角</button></div>
<div class="layout"><div class="stage"><canvas id="surface" aria-label="规划余量、参考余量与成功率三维曲面。精确数据另见下方表格。"></canvas>
<div class="legend"><span><i class="dot"></i>49 个实测组合</span><span><i class="dot base"></i>原基线 4.2 / 4.0 m</span><span><i class="dot peak"></i>本轮最高 1.2 / 4.0 m</span><span>网格面仅连接采样点</span></div></div>
<aside class="aside"><h2 id="cardTitle">当前选中组合</h2><div class="hint">规划余量 / 参考余量</div><div class="number" id="margins"></div>
<dl><dt>完整任务成功率</dt><dd id="rate"></dd><dt>成功数 / 固定分母</dt><dd id="count"></dd><dt>95% Wilson 区间</dt><dd id="interval"></dd></dl>
<div class="details"><h2>失败计数 · 全部 200 张</h2><dl id="failureDetails"></dl></div><div class="hint">救回与丢失均相对于原基线，在完全相同的 200 张地图上逐图比较。</div></aside></div>
<p class="note">横轴：规划余量 pathClearance / m；纵轴：参考余量 desiredClear / m；高度：完整任务成功率 / %。两项余量均为各向同性，高度策略与控制器保持 2.3.1。43 张初始不安全地图在所有组合中保留。中间参数未经仿真，网格面不是经过验证的连续函数。</p>
<details class="sources"><summary>查看全部 49 个实测组合与源数据</summary><div class="tablewrap"><table><thead><tr><th>组合</th><th>规划/m</th><th>参考/m</th><th>成功/200</th><th>总体成功率</th><th>初始安全成功率</th><th>规划失败</th><th>超时</th><th>安全未解决</th><th>重规划失败</th><th>碰撞</th></tr></thead><tbody id="rows"></tbody></table></div></details>
<div class="footer"><a href="grid_summary.csv">49 组汇总 CSV</a><a href="all_results.csv">9800 条原始结果 CSV</a><a href="figures/01_success_surface_all200.png">三维图 PNG</a><a href="figures/01_success_surface_all200.pdf">分享 PDF</a><a href="figures/01_success_surface_all200.fig">MATLAB 可编辑 FIG</a><a href="REPORT.md">实验说明</a></div></main>
<script>
'use strict';const data=__DATA__,xs=[.8,1.2,1.8,2.5,3.5,4.2,5.6],ys=[.8,1.2,1.8,2.5,3.5,4,4.4];
const canvas=document.getElementById('surface'),ctx=canvas.getContext('2d');let mode='all',az=-.82,el=.47,zoom=1,drag=null,hits=[],selected=data.find(d=>d.pathClearance===4.2&&d.desiredClear===4);const peak=data.reduce((a,b)=>b.success>a.success?b:a);
const percent=d=>mode==='all'?d.successPercent:d.safeSuccessPercent;
function point(x,y,z){return [(x-.8)/4.8*2-1,(y-.8)/3.6*2-1,z/100*1.65]}
function projected(p){let u=Math.cos(az)*p[0]-Math.sin(az)*p[1],v=Math.sin(az)*p[0]+Math.cos(az)*p[1];let depth=Math.cos(el)*v-Math.sin(el)*p[2],vertical=Math.sin(el)*v+Math.cos(el)*p[2];let s=Math.min(canvas.clientWidth*.30,canvas.clientHeight*.38)*zoom;return [canvas.clientWidth*.5+u*s,canvas.clientHeight*.72-vertical*s,depth]}
function line(a,b,color,width=1,dash=[]){let u=projected(a),v=projected(b);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.beginPath();ctx.moveTo(u[0],u[1]);ctx.lineTo(v[0],v[1]);ctx.stroke();ctx.setLineDash([])}
function label(p,text,dx=0,dy=0,align='center',color='#677689'){let u=projected(p);ctx.fillStyle=color;ctx.font='12px Segoe UI, Microsoft YaHei, sans-serif';ctx.textAlign=align;ctx.fillText(text,u[0]+dx,u[1]+dy)}
function draw(){let ratio=window.devicePixelRatio||1,w=canvas.clientWidth,h=canvas.clientHeight;canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,w,h);hits=[];
for(let z=0;z<=100;z+=20){line(point(.8,.8,z),point(.8,4.4,z),'#e8edf4');label(point(.8,4.4,z),z+'%',-10,4,'right')}
for(const x of xs){line(point(x,.8,0),point(x,4.4,0),'#e5ebf3');label(point(x,.8,0),x.toFixed(1),0,20)}
for(const y of ys){line(point(.8,y,0),point(5.6,y,0),'#e5ebf3');label(point(5.6,y,0),y.toFixed(1),18,7,'left')}
line(point(.8,.8,0),point(5.6,.8,0),'#8a99ab',1.3);line(point(5.6,.8,0),point(5.6,4.4,0),'#8a99ab',1.3);line(point(.8,4.4,0),point(.8,4.4,100),'#8a99ab',1.3);
label(point(3.2,.8,0),'规划余量 / m',0,49);label(point(5.6,2.6,0),'参考余量 / m',w<500?5:60,15,w<500?'center':'left');label(point(.8,4.4,100),'成功率',-8,-22,'right');
if(mode==='all'){line(point(.8,.8,78.5),point(5.6,.8,78.5),'#99a4b3',1,[5,4]);label(point(.8,.8,78.5),'初始安全上限 78.5%',-5,-12,'right')}
const byKey=new Map(data.map(d=>[`${d.pathIndex},${d.referenceIndex}`,d])),items=[];
for(let a=1;a<7;a++)for(let b=1;b<7;b++){let ds=[[a,b],[a+1,b],[a+1,b+1],[a,b+1]].map(([i,j])=>byKey.get(`${i},${j}`));let p=ds.map(d=>projected(point(d.pathClearance,d.desiredClear,percent(d))));items.push({kind:'face',depth:p.reduce((s,v)=>s+v[2],0)/4,p,z:ds.reduce((s,d)=>s+percent(d),0)/4})}
for(const d of data){let p=projected(point(d.pathClearance,d.desiredClear,percent(d)));items.push({kind:'point',depth:p[2]-.001,p,d});hits.push({p,d})}
items.sort((a,b)=>b.depth-a.depth);for(const item of items){if(item.kind==='face'){let f=item.z/100;ctx.beginPath();item.p.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.closePath();ctx.fillStyle=`rgb(${Math.round(226-186*f)},${Math.round(241-129*f)},${Math.round(252-55*f)})`;ctx.fill();ctx.strokeStyle='#718aa7';ctx.lineWidth=.65;ctx.stroke()}else{let d=item.d,base=d.pathClearance===4.2&&d.desiredClear===4,highest=d===peak;ctx.beginPath();ctx.arc(item.p[0],item.p[1],base||highest?5.8:3.2,0,Math.PI*2);ctx.fillStyle=base?'#e98821':highest?'#0d8c59':'#174578';ctx.fill();if(d===selected){ctx.beginPath();ctx.arc(item.p[0],item.p[1],9,0,Math.PI*2);ctx.strokeStyle='#17293e';ctx.lineWidth=1.4;ctx.stroke()}}}}
function show(d){selected=d;document.getElementById('cardTitle').textContent=d===peak?'本轮最高组合':d.pathClearance===4.2&&d.desiredClear===4?'原基线组合':'当前选中组合';document.getElementById('margins').textContent=`${d.pathClearance.toFixed(1)} / ${d.desiredClear.toFixed(1)} m`;document.getElementById('rate').textContent=percent(d).toFixed(2)+'%';document.getElementById('count').textContent=`${d.success} / ${mode==='all'?200:157}`;let lo=mode==='all'?d.wilsonLow:d.safeWilsonLow,hi=mode==='all'?d.wilsonHigh:d.safeWilsonHigh;document.getElementById('interval').textContent=lo.toFixed(2)+'–'+hi.toFixed(2)+'%';let fields=[['初始不安全',d.initialUnsafe],['规划失败',d.planningFailed],['飞行超时',d.timeout],['安全未解决',d.safetyUnresolved],['重规划失败',d.replanFailed],['碰撞',d.collision],['越界 / 无效状态',d.boundaryViolation+' / '+d.invalidState],['救回 / 丢失',d.rescued+' / '+d.lost]];document.getElementById('failureDetails').innerHTML=fields.map(([k,v])=>`<dt>${k}</dt><dd>${v}</dd>`).join('');draw()}
for(const m of ['all','safe'])document.getElementById(m).onclick=()=>{mode=m;document.getElementById('all').setAttribute('aria-pressed',m==='all');document.getElementById('safe').setAttribute('aria-pressed',m==='safe');show(selected)};
document.getElementById('peak').onclick=()=>show(peak);document.getElementById('baseline').onclick=()=>show(data.find(d=>d.pathClearance===4.2&&d.desiredClear===4));
document.getElementById('reset').onclick=()=>{az=-.82;el=.47;zoom=1;draw()};canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)};canvas.onpointerup=()=>drag=null;
canvas.onpointermove=e=>{if(drag){az+=(e.clientX-drag[0])*.007;el=Math.max(.15,Math.min(1.25,el-(e.clientY-drag[1])*.005));drag=[e.clientX,e.clientY];draw();return}let r=canvas.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top,best=null,distance=196;for(const hit of hits){let dist=(hit.p[0]-x)**2+(hit.p[1]-y)**2;if(dist<distance){distance=dist;best=hit.d}}if(best&&best!==selected)show(best)};
canvas.onwheel=e=>{e.preventDefault();zoom=Math.max(.65,Math.min(1.7,zoom*Math.exp(-e.deltaY*.001)));draw()};new ResizeObserver(draw).observe(canvas);
document.getElementById('rows').innerHTML=data.map(d=>`<tr><td>${d.configId}</td><td>${d.pathClearance.toFixed(1)}</td><td>${d.desiredClear.toFixed(1)}</td><td>${d.success}/200</td><td>${d.successPercent.toFixed(1)}%</td><td>${d.safeSuccessPercent.toFixed(2)}%</td><td>${d.planningFailed}</td><td>${d.timeout}</td><td>${d.safetyUnresolved}</td><td>${d.replanFailed}</td><td>${d.collision}</td></tr>`).join('');show(selected);
</script></html>'''

def render(study):
    study=Path(study)
    audit=json.loads((study/'raw_mat_audit.json').read_text(encoding='utf-8'))
    assert audit['passed'] and audit['records']==9800 and audit['baselineExactReplays']==200
    with (study/'grid_summary.csv').open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
    keep=['configId','pathIndex','referenceIndex','pathClearance','desiredClear','success','successPercent',
          'safeSuccessPercent','wilsonLow','wilsonHigh','safeWilsonLow','safeWilsonHigh','initialUnsafe',
          'planningFailed','timeout','safetyUnresolved','replanFailed','collision','boundaryViolation','invalidState','rescued','lost']
    data=[{k:(r[k] if k=='configId' else float(r[k])) for k in keep} for r in rows]
    for d in data:
        for k in ['pathIndex','referenceIndex','success','initialUnsafe','planningFailed','timeout','safetyUnresolved','replanFailed','collision','boundaryViolation','invalidState','rescued','lost']:d[k]=int(d[k])
    assert len(data)==49 and len({d['configId'] for d in data})==49
    for d in data:
        assert abs(d['successPercent']-100*d['success']/200)<1e-10
        assert abs(d['safeSuccessPercent']-100*d['success']/157)<1e-10
    document=HTML.replace('__DATA__',json.dumps(data,ensure_ascii=False,allow_nan=False))
    metadata=dict(source='grid_summary.csv',sourceSha256=hashlib.sha256((study/'grid_summary.csv').read_bytes()).hexdigest(),
        rawAudit='raw_mat_audit.json',samplePoints=49,denominators=[200,157],
        metric='Complete original flight success criterion; fixed-map exploratory scan.')
    document=document.replace('</html>','<script type="application/json" id="source-metadata">'+json.dumps(metadata,ensure_ascii=False)+'</script></html>')
    path=study/'margin_success_surface.html';path.write_text(document,encoding='utf-8')
    print(path)
if __name__=='__main__':render(sys.argv[1])

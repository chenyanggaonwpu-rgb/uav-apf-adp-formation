from __future__ import annotations
from pathlib import Path
import os,sys,json,csv,math,statistics,collections,importlib.util,shutil,datetime
import numpy as np
ROOT=Path(__file__).resolve().parent;PROJECT=next(p for p in ROOT.parents if (p/'unified_actor3d').is_dir());RUN=PROJECT/'results/unified_actor_20261029'
TMP=ROOT/'_work/tmp';TMP.mkdir(parents=True,exist_ok=True);os.environ['TEMP']=os.environ['TMP']=str(TMP);sys.dont_write_bytecode=True
import layout_base as layout
from reportlab.lib.colors import HexColor,black,white
layout.ROOT=ROOT;layout.PROJECT=PROJECT;layout.FIG=ROOT/'figures';layout.FIG.mkdir(exist_ok=True)
EVIDENCE={
 'E01':('统一控制器与源码说明',PROJECT/'docs/KNOWN_MAP_UNIFIED_ACTOR.md'),
 'E02':('唯一冻结模型与训练绑定',PROJECT/'controller_assets/frozen_actor_20261015/asset_manifest.json'),
 'E03':('全量实验方案与参数',RUN/'protocol.json'),
 'E04':('3840次逐任务结果',RUN/'trials.csv'),
 'E05':('Python独立动力学复核',RUN/'independent_audit.csv'),
 'E06':('同图共同成功配对统计',RUN/'paired_summary.json'),
 'E07':('外扰恢复记录',RUN/'recovery.csv'),
 'E08':('五机串行完整周期耗时',RUN/'serial_timing/serial_timing.csv'),
 'E09':('原始飞行记录与全状态轨迹',RUN/'flight'),
 'E10':('源码与模型指纹及完成记录',RUN/'completion.json'),
 'E11':('模型文件与默认入口GitHub同步记录',RUN/'github_source_receipt.json'),
 'E12':('现象判据与独立分析源码',PROJECT/'experiments3d/unified_actor_20261029/analysis_protocol.json'),
 'E13':('动态现象及独立几何复核',RUN/'动态现象与依据.md'),
 'E14':('计算异常与同输入串行重试记录',RUN/'execution_retry_registry.json'),
 'E15':('逐步CSV轨迹与记录索引',RUN/'trace_index.json'),
 'E16':('安全层同图结局和代价分析',RUN/'安全层同图对照分析.md'),
}
layout.EVIDENCE=EVIDENCE;XSL=layout.XSL;EQS=layout.EQS
mi,mn,mo,row,sub,frac,vec,par,norm=layout.mi,layout.mn,layout.mo,layout.row,layout.sub,layout.frac,layout.vec,layout.par,layout.norm
EQS['actor_observation']=row(mi('o'),mo('='),vec(frac(row(mi('g'),mo('−'),mi('p')),mn(25)),frac(mi('v'),mn(3)),frac(sub(mi('d'),mn(26)),mn(12))))
EQS['actor_waypoint']=row(sub(mi('q'),mi('A')),mo('='),mi('p'),mo('+'),mn(8),frac(row(mi('π'),par(mi('o'))),row(mi('max'),par(row(mn(1),mo(','),norm(row(mi('π'),par(mi('o')))))))))
EQS['parameter_count']=row(mi('N'),mo('='),mn(32),mo('×'),mn(64),mo('+'),mn(64),mo('+'),mn(64),mo('×'),mn(64),mo('+'),mn(64),mo('+'),mn(64),mo('×'),mn(3),mo('+'),mn(3),mo('='),mn(6467))
EQS['offline_reward']=row(mi('r'),mo('='),sub(mi('D'),mi('before')),mo('−'),sub(mi('D'),mi('after')),mo('−'),mn('0.2'),mi('Δt'),mo('−'),mn('0.5'),sub(mi('N'),mi('safety')),mo('−'),mn('0.01'),norm(mi('a')),mo('²'),mi('Δt'),mo('+'),sub(mi('r'),mi('terminal')))
layout.EQS=EQS
COLORS=[HexColor(x) for x in ['#526A79','#B67D4A','#246D6C','#7D5A8B','#AD4141']]
FACTS={};ROWS=[];AUDITS={}
def fmt(v,n=2):return '—' if v is None or not math.isfinite(float(v)) else f'{v:.{n}f}'
def sel(campaign=None,method=None,safety=None,policy=None,**kw):
    return [r for r in ROWS if (campaign is None or r['campaign']==campaign) and (method is None or r['method']==method) and (safety is None or r['safety']==safety) and (policy is None or r['routePolicy']==policy) and all(r[k]==v for k,v in kw.items())]
def stat(rows):
    good=[r for r in rows if r['success']]
    return dict(n=len(rows),ok=len(good),calls=sum(r['actorCalls'] for r in rows),active=sum(r['actorCalls']>0 for r in rows),
        failures=dict(collections.Counter(r['reason'] for r in rows if not r['success'])),
        obs=min((r['minObstacleDistance'] for r in rows if r['minObstacleDistance'] is not None),default=None),
        pair=min((r['minPairDistance'] for r in rows if r['minPairDistance'] is not None),default=None),
        time=statistics.mean(r['flightSeconds'] for r in good) if good else None,
        path=statistics.mean(r['pathLength'] for r in good) if good else None,
        rms=statistics.mean(r['formationRms'] for r in good) if good else None,
        stall=sum(r['maxStallSeconds']>=3 for r in rows),sway=sum(AUDITS[r['id']].get('lateralSway',False) for r in rows),
        maxCommand=max((r['maxCommand'] for r in rows if r['maxCommand'] is not None),default=None),
        maxAcceleration=max((r['maxActualAcceleration'] for r in rows if r['maxActualAcceleration'] is not None),default=None))
def ratio(rows):s=stat(rows);return f"{s['ok']}/{s['n']}"
def arm(m,s):return ('A1' if m=='actor' else 'A0')+('S1' if s else 'S0')
def mdtable(headers,rows):return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,r))+' |' for r in rows])
def four_table(campaign,cost=False):
    values=[]
    for m,s in [('none',False),('actor',False),('none',True),('actor',True)]:
        a=stat(sel(campaign,m,s));r=[arm(m,s),f"{a['ok']}/{a['n']}",fmt(a['obs'],3),fmt(a['pair'],3),str(a['active'])]
        if cost:r.extend([fmt(a['time']),fmt(a['path']),fmt(a['rms'],3)])
        values.append(r)
    return mdtable(['组别','成功/总数','最小障碍净空 m','最小机距 m','Actor介入任务']+(['成功均时 s','成功均路长 m','成功均RMS m'] if cost else []),values)
def paired(campaign,first='actor_S0',second='actor_S1',policy=None):
    return next(r for r in FACTS['paired'] if r['campaign']==campaign and r['first']==first and r['second']==second and r['routePolicy']==policy)
def recovery_table():
    data=[]
    for amplitude in [.1,.3,.5,1.]:
        r=next(r for r in FACTS['recovery'] if r['kind']=='gust' and r['method']=='actor' and r['safety'] and r['amplitude']==amplitude)
        data.append([str(amplitude),f"{r['recovered']}/{r['trials']}",fmt(r['meanConfirmedSeconds'],2)])
    return mdtable(['脉冲幅值 m/s²','5秒内恢复/总数','已恢复任务平均确认时间 s'],data)
def pair_text(p):
    d=p['difference'];return f"两组共同成功任务为 {p['commonSuccesses']} 次。后一组减前一组的平均差：完成时间 {fmt(d['flightSeconds'],3)} 秒，长机路径 {fmt(d['pathLength'],3)} 米，队形 RMS {fmt(d['formationRms'],4)} 米。另有 {p['rescued']} 次由失败变成功、{p['regressions']} 次由成功变失败。"
def read_result(id):
    spec=importlib.util.spec_from_file_location('mat5',PROJECT/'experiments3d/unified_actor_20261029/read_mat5.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    v=m.variable(RUN/'flight'/(id+'.mat'),'result')
    def n(key):shape,data=m.numeric(v,key);return np.array(data).reshape(shape,order='F')
    count=next(r['steps'] for r in ROWS if r['id']==id)
    return dict(state=n('history.state').reshape(6,-1,count,order='F'),command=n('history.command').reshape(3,-1,count,order='F'),
        obs=n('history.obstacleClearance').ravel(),pair=n('history.pairDistance').ravel(),time=n('history.t').ravel(),
        safety=n('history.safetyTriggered').ravel(),initial=n('initialFleetState'))
def figure(name,height,draw):return layout.figure_file(name,height,lambda canvas,width,h:draw(canvas))
def plot_lines(c,rect,x,y_arrays,labels,yr,xlabel,ylabel):
    xx=np.asarray(x);lo,hi=float(xx.min()),float(xx.max());hi=max(hi,lo+1e-3)
    xt=np.linspace(lo,hi,5);yt=np.linspace(yr[0],yr[1],5)
    fx,fy=layout.axes(c,rect,(lo,hi),yr,xt,yt,xlabel,'')
    layout.txt(c,rect[0],rect[1]+rect[3]+31,ylabel,9,layout.GRAY)
    for j,(y,label) in enumerate(zip(y_arrays,labels)):
        ix=np.linspace(0,len(xx)-1,min(len(xx),700)).astype(int);points=[(fx(xx[i]),fy(float(y[i]))) for i in ix]
        layout.polyline(c,points,COLORS[j%len(COLORS)],1.2)
        layout.txt(c,rect[0]+j*rect[2]/len(labels),rect[1]+rect[3]+14,label,8.0,COLORS[j%len(COLORS)])
def make_figures():
    def flow(c):
        labels=[(18,198,115,44,['地图与起终点','全局分层规划']),(171,198,116,44,['监督器选择航点','全局航点或临时航点']),(324,198,110,44,['水平 V 形参考','APF 与机间作用']),
            (18,78,115,44,['3 m/s² 模长限幅','候选加速度']),(171,78,116,44,['预测修正和制动检查','安全开关 S']),(324,78,110,44,['真实质点积分','指令加外部扰动'])]
        labels.append((171,278,116,44,['同一个冻结 Actor','局部32维观测']))
        for x,y,w,h,ll in labels:
            c.setStrokeColor(HexColor('#687782'));c.setFillColor(white);c.rect(x,y,w,h,fill=1,stroke=1)
            for j,t in enumerate(ll):layout.txt(c,x+w/2,y+h-17-j*14,t,9,black,'center')
        for x1,y1,x2,y2 in [(133,220,171,220),(287,220,324,220),(379,198,379,153),(379,153,75,153),(75,153,75,122),(133,100,171,100),(287,100,324,100)]:layout.arrow(c,x1,y1,x2,y2,COLORS[0])
        layout.arrow(c,229,278,229,242,COLORS[2]);layout.txt(c,245,258,'接管时',8.5)
        for x1,y1,x2,y2 in [(434,100,462,100),(462,100,462,300),(462,300,287,300),
            (462,260,310,260),(310,260,310,220),(310,220,287,220),(462,220,434,220)]:layout.arrow(c,x1,y1,x2,y2,COLORS[2])
        layout.txt(c,462,316,'测量反馈',8.2,COLORS[2],'right')
        layout.txt(c,260,168,'选中的航点驱动基础控制',9)
        layout.txt(c,225,38,'真实状态评价安全  测量位置返回全部反馈入口',9,black,'center')
        layout.txt(c,225,20,'反馈加位置噪声  物理加外部加速度  在线不更新模型',9,black,'center')
    figure('控制信息流',345,flow)
    svg='<svg xmlns="http://www.w3.org/2000/svg" width="1080" height="650" viewBox="0 0 1080 650"><defs><marker id="a" markerWidth="10" markerHeight="10" refX="8" refY="3" orient="auto"><path d="M0 0 L8 3 L0 6 Z" fill="#526A79"/></marker></defs><g font-family="SimHei, sans-serif" font-size="22" fill="#111">'
    boxes=[(30,150,'地图与分层规划'),(360,150,'监督器选择航点'),(690,150,'编队参考与 APF'),(30,400,'指令模长限幅'),(360,400,'预测修正与制动检查'),(690,400,'真实质点与外部扰动'),(360,10,'同一个冻结 Actor')]
    for x,y,t in boxes:svg+=f'<rect x="{x}" y="{y}" width="280" height="90" fill="white" stroke="#687782"/><text x="{x+140}" y="{y+54}" text-anchor="middle">{t}</text>'
    svg+='</g><g stroke="#526A79" stroke-width="3" fill="none" marker-end="url(#a)"><path d="M310 195 H350"/><path d="M640 195 H680"/><path d="M500 100 V140"/><path d="M830 240 V320 H170 V390"/><path d="M310 445 H350"/><path d="M640 445 H680"/><path d="M970 445 H1020 V55 H650"/><path d="M1020 195 H980"/><path d="M1020 125 H665 V195 H650"/></g><g font-family="SimHei, sans-serif" font-size="22"><text x="270" y="300">选中的临时或全局航点</text><text x="230" y="590">位置反馈加噪声  真实状态独立评价</text></g></svg>'
    (ROOT/'figures/控制信息流_可编辑.svg').write_text(svg,encoding='utf-8')
    def results(c):
        keys=['repair','dynamic','integration','single'];names=['原安全层地图','APF 动态场景','五机接入','单机诊断']
        for j,(key,label) in enumerate(zip(keys,names)):
            y=210-j*50;layout.txt(c,14,y+13,label,10)
            for k,m in enumerate(['none','actor']):
                g=stat(sel(key,m,True));x=136;yy=y-k*15;c.setFillColor(COLORS[k]);c.rect(x,yy,260*g['ok']/g['n'],10,fill=1,stroke=0)
                layout.txt(c,403,yy+1,f"{g['ok']}/{g['n']}",9,COLORS[k])
        layout.txt(c,136,17,'上条 A0S1  下条 A1S1  均使用同一安全层',9)
    figure('同图结果',258,results)
    def factorial(c):
        for j,policy in enumerate(['global','local_apf']):
            x=28+j*232;layout.txt(c,x,229,'全局规划开启' if policy=='global' else '局部 APF 压力场景',11)
            for k,m in enumerate(['none','rule','actor']):
                g=stat(sel('factorial',m,True,policy));yy=176-k*48;c.setFillColor(COLORS[k]);c.rect(x,yy,180*g['ok']/g['n'],16,fill=1,stroke=0)
                layout.txt(c,x,yy+22,{'none':'无脱困策略','rule':'规则脱困','actor':'统一 Actor'}[m]+f"  {g['ok']}/144",9)
        layout.txt(c,28,19,'同 144 张地图  所有组合保留  全封堵反例单独解释',9)
    figure('六组合结果',260,factorial)
    def robust(c):
        levels=[.02,.05,.1];series=[]
        for m,s in [('none',False),('actor',False),('none',True),('actor',True)]:
            series.append([100*stat(sel('robust',m,s,kind='noise',noiseStd=n))['ok']/36 for n in levels])
        plot_lines(c,(58,48,362,170),np.array(levels),series,['A0S0','A1S0','A0S1','A1S1'],(0,105),'位置噪声每轴标准差 m','成功比例 %')
    figure('噪声等级',265,robust)
    # All plotted state/control panels come from the same selected formal run.
    selected=sel('integration','actor',True,mapID='window_63001')
    if selected and selected[0]['steps']:
        r=selected[0];d=read_result(r['id']);state=d['state'];t=d['time']
        def case(c):
            plot_lines(c,(58,240,360,115),t,[state[2,i,:] for i in range(5)],['长机','僚机2','僚机3','僚机4','僚机5'],(0,30),'时间 s','高度 m')
            ymax=max(3.5,float(np.linalg.norm(d['command'],axis=0).max())+.2)
            plot_lines(c,(58,42,360,105),t,[np.linalg.norm(d['command'][:,i,:],axis=0) for i in range(5)],['长机','僚机2','僚机3','僚机4','僚机5'],(0,ymax),'时间 s','指令模长 m/s²')
        figure('Actor脱困时序',415,case)
    noisy=sel('robust','actor',True,mapID='M01',kind='noise',noiseStd=.1)
    if noisy:
        d=read_result(sorted(noisy,key=lambda x:x['id'])[0]['id']);t=d['time']
        def noise(c):
            plot_lines(c,(58,228,360,112),t,[d['command'][1,0,:],d['state'][4,0,:]],['长机 Y 加速度指令','长机 Y 速度'],(-3.3,3.3),'时间 s','m/s² 或 m/s')
            ymax=max(2,float(np.max([d['obs'].max(),d['pair'].max()]))*1.05)
            plot_lines(c,(58,39,360,100),t,[d['obs'],d['pair']],['最小障碍净空','最小机间距离'],(0,ymax),'时间 s','距离 m')
        figure('噪声控制与轨迹',393,noise)
    (ROOT/'figure_source_records.json').write_text(json.dumps(dict(actor='integration_window_63001_B_actor_S1',noise=noisy[0]['id'] if noisy else None),ensure_ascii=False,indent=2),encoding='utf-8')
    # Select by a fixed illustration rule: first repair-map failure rescued by
    # safety, while keeping every map in the aggregate comparisons.
    off={(r['mapID'],r['tag']):r for r in sel('repair','actor',False)}
    rescues=sorted([r for r in sel('repair','actor',True) if r['success'] and not off[(r['mapID'],r['tag'])]['success']],key=lambda r:r['id'])
    if rescues:
        good=rescues[0];bad=off[(good['mapID'],good['tag'])];a=read_result(bad['id']);b=read_result(good['id'])
        def overlay(c):
            fx,fy=layout.axes(c,(58,205,360,130),(0,80),(0,80),[0,20,40,60,80],[0,20,40,60,80],'X m','')
            layout.txt(c,36,335,'Y m',8.5,layout.GRAY,'right')
            for d,color in [(b,COLORS[2]),(a,COLORS[0])]:
                ix=np.linspace(0,len(d['time'])-1,min(len(d['time']),500)).astype(int)
                layout.polyline(c,[(fx(d['state'][0,0,k]),fy(d['state'][1,0,k])) for k in ix],color,1.4)
            layout.txt(c,58,360,'同一地图 '+good['mapID']+'  长机俯视轨迹',10)
            layout.txt(c,58,346,'灰色 A1S0  绿色 A1S1',9)
            max_t=max(a['time'][-1],b['time'][-1]);max_d=max(a['obs'].max(),b['obs'].max())*1.04
            fx,fy=layout.axes(c,(58,42,360,108),(0,max_t),(0,max_d),np.linspace(0,max_t,5),np.linspace(0,max_d,5),'时间 s','全队扫掠最小净空 m')
            for d,color in [(a,COLORS[0]),(b,COLORS[2])]:
                ix=np.linspace(0,len(d['time'])-1,min(len(d['time']),600)).astype(int)
                layout.polyline(c,[(fx(d['time'][k]),fy(d['obs'][k])) for k in ix],color,1.2)
            layout.line(c,58,fy(.6),418,fy(.6),COLORS[4],.8,[3,2])
        figure('安全层同图轨迹',385,overlay)
        (ROOT/'safety_figure_records.json').write_text(json.dumps(dict(withoutSafety=bad['id'],withSafety=good['id']),ensure_ascii=False,indent=2),encoding='utf-8')

def conclusion():
    g0=stat(sel('factorial','none',True,'global'));g1=stat(sel('factorial','actor',True,'global'))
    l0=stat(sel('factorial','none',True,'local_apf'));l1=stat(sel('factorial','actor',True,'local_apf'));lr=stat(sel('factorial','rule',True,'local_apf'))
    local=f"Actor在局部停滞场景增加了 {l1['ok']-l0['ok']} 次成功。" if l1['ok']>l0['ok'] else 'Actor在这批局部场景没有增加成功次数。'
    global_result='全局规划开启时，Actor没有增加成功次数。' if g1['ok']==g0['ok'] else f"全局规划开启时，Actor的净增成功次数为 {g1['ok']-g0['ok']}。"
    rule_result=f"规则脱困比Actor多完成 {lr['ok']-l1['ok']} 次。" if lr['ok']>l1['ok'] else ('Actor与规则脱困完成次数相同。' if lr['ok']==l1['ok'] else f"Actor比规则脱困多完成 {l1['ok']-lr['ok']} 次。")
    return f"在同一组144张地图上，全局规划开启时，基础方案成功 {g0['ok']} 次，统一 Actor 方案成功 {g1['ok']} 次；关闭全局路线、只靠局部 APF 时，基础方案成功 {l0['ok']} 次，Actor 方案成功 {l1['ok']} 次，规则脱困成功 {lr['ok']} 次。{local}{global_result}{rule_result}"

def student_text():
    pages=[]
    def p(title,body):pages.append(('## '+title+'\n\n' if title else '')+body.strip())
    p('',f'''# 无人机编队智能避障项目整体报告

学生阅读版 统一 Actor 重验版

版本 2.3.4-unified-actor-20261029  整理日期 2026年10月30日

本项目让一架长机带领四架僚机，在已知三维障碍地图中飞到目标位置。五架机既要绕开障碍，也要控制彼此距离，还要在终点附近减速、恢复队形。程序把这些要求落实为规划、编队参考、局部控制、学习脱困和安全检查，再把每一步的真实运动保存下来。

本版所有地图默认使用同一个训练好的 Actor，共6467个固定参数。Actor 在长机持续卡住时提出临时航点，帮助基础控制改变局部运动方向。它在地图之间共享，飞行中没有重新学习。

{conclusion()}

本次共重跑3840次已声明任务，覆盖安全层消融、APF退化、噪声与外扰、五机接入、六组合对照和单机诊断。两套独立程序核对了保存的动力学；原始失败记录也进入统计。报告下面从最基本的状态和输入讲起，再解释这些结果说明了什么。

@source E01 E02 E03 E04 E05''')
    p('01 先把飞行任务说清楚','''可以把长机理解为领队，四架僚机按前后两排跟随。每架机都知道自己的位置和速度。地图给出了球体、有限高度圆柱和箱体的位置及大小，程序在这个已知环境中计算路线。

“到达终点”包含四个条件：长机距离目标不超过1米，四架僚机相对各自动态参考的最大误差不超过1米，五架机的最大速度不超过0.25米/秒，参考位置没有被障碍挡住。这些条件还要持续1秒。只看长机从目标旁边飞过，不能算整队完成任务。

工作空间为80米×80米×30米。基础高度层为5、10、15、20、25米。速度模长上限为3米/秒，加速度指令模长上限为3米/秒平方。真实轨迹每0.05秒更新一次，也就是每个仿真秒计算20步。

一个普通场景中，规划先给长机一串航点；长机沿航点走，僚机跟随动态参考。遇到局部卡住，Actor 接管临时航点。安全模块在每一步决定本次加速度能否执行。最后，独立的真实运动检查器核对五架机实际经过的整段运动。

这些模块处理的是不同问题。路线通不通、局部方向能否摆脱卡住、五架机能否跟上、安全距离是否保持，需要分别看记录。

@source E01 E03''')
    p('02 位置速度和加速度怎样变成轨迹','''位置是三维坐标，例如[20,40,15]表示在水平面内的两个坐标为20米、40米，高度15米。速度也是三维向量；[3,0,0]表示沿世界坐标X轴每秒前进3米。加速度指令决定下一步速度如何变化。

程序采用二阶质点模型。每一步先用加速度更新速度，再用更新后的速度推进位置。下面的Δt就是0.05秒，u是最终执行指令，d是外部加速度。sat表示按向量总长度限幅。

@eq motion_student

例如从静止开始，沿X轴施加3米/秒平方的指令，一步后的速度增加0.15米/秒，位置前进0.0075米。这里没有直接修改位置来绕障，也没有在安全检查失败时把运动暂停。

三个方向共同受到模长限制。若候选输入为[3,3,0]，它的长度约为4.24，超过3，所以要整体缩小到约[2.12,2.12,0]。三个轴分别限制到3，会允许总加速度超过3，和本项目的实现不一致。

外扰加在命令限幅之后。因此“指令不超过3”和“实际总加速度不超过3”是两件事。本轮分别记录了两者。

@source E01 E05 E09''')
    p('03 一次循环中各模块怎样配合','''程序不会让五架机轮流移动。它们在同一个时间步读取状态、计算输入，随后同步推进。这样检查机间相对运动时，两架机处于同一个时间区间。

@figure 控制信息流 | 图1 已知地图控制信息流 可编辑SVG与本图一起保存

地图和目标决定全局航点。监督器根据长机最近一段运动判断是否卡住，满足条件才向同一个 Actor 请求临时航点。选中的航点送入长机基础控制，僚机则使用动态编队参考。

基础控制由目标跟踪、障碍作用和机间作用共同组成。命令先限幅，再交给预测修正和制动检查，最后进入真实模型。反馈位置可以带噪声，真实状态留给物理推进与结果评价。

图中的每个箭头都对应实际调用。Actor 不直接驱动五架机，也不修改真实轨迹。它改变长机此刻追逐的局部目标，后续控制仍负责把这个意图变成加速度。

@source E01 E03''')
    p('04 障碍距离为什么要算到表面','''无人机距离一个球体有多近，应计算到球面，而不是到球心。例如球心距离5米、半径4米，实际表面净空只有1米。

@eq sphere

有限高度圆柱既有侧面，也有顶面和底面。在柱子顶部边缘附近，最近点可能从侧面切换到顶面，或落在棱边上。程序把水平径向距离和垂向距离共同计算，不能只用水平圆来代表整个圆柱。

箱体用于构造墙体、窗口和全封堵反例。箱体的表面距离同样参加安全检查。窗口看起来能让长机通过，并不意味着水平展开的四架僚机都能按原参考通过。

本版障碍硬安全距为0.6米，机间硬安全距为1米。障碍净空是到障碍表面的距离，机间距离是两架机位置之差的长度。两列数据分别保存。

代码把突破硬安全距的情况判为碰撞；本轮还额外统计障碍表面距离不大于0的情况。阅读结果时，应先认清这个判定口径，再看失败发生在哪架机、哪个时刻。

@source E03 E04 E09 E12''')
    p('05 全局规划怎样选出一条空间路线','''全局规划相当于先在地图上找出一串可走的路标。每个高度层上铺设1米分辨率的水平网格，节点与同层的八个相邻节点连接；相邻高度层之间使用垂直连接。整条边、起终点接入段和简化后的长线段都要检查障碍净空。

路线代价包含三维路长和累计升降量。垂直运动权重为1，表示同样的几何路长下，程序还会额外考虑不必要的升降。

@eq route

基础五层有81×81×5=32805个网格容量。这个数量包含被障碍和边界剔除的节点，实际可用节点要看具体地图。相比每一米高度都铺完整三维网格，有限层能减少规模，但也可能错过两层之间的窄通道。

为处理缺层窗口，当前程序在固定层搜索失败时，按障碍自由高度区间补充有限个高度层，最多新增8层。补层有预算，并没有连续搜索全部空间。

全局规划失败时，本报告保留该任务。它说明当前图结构和净空条件没有找到路线，不能直接等同于现实空间不存在任何路径。

@source E01 E03 E09''')
    p('06 APF 怎样给出局部绕障方向','''APF 是人工势场。可以把目标跟踪理解为把无人机拉向当前航点，把障碍斥力理解为从障碍表面向外推。靠近障碍时，控制器增加障碍作用的权重。

仅沿障碍法向往外推，可能让无人机停在目标与障碍正中的位置。程序因此加入切向分量，让运动有机会沿障碍表面绕开。三维切向由“去航点的方向减去它在表面法向上的分量”得到。

@eq tangent

如果无人机正对球心，去航点的方向与球面法向接近共线，投影就接近零。此时直接归一化会放大数值误差，甚至产生非法数值。源码使用1e-6的投影长度阈值；第一次投影不足时，改用原始目标方向再次投影，仍不足就标记退化。

退化标记会带来额外减速，全局路线条件下还保留重规划，持续停滞时可进入 Actor 脱困。程序没有随意选一个三维旋转方向。因此，正对球体、向左或向上偏极小距离，可能触发不同的局部运动，这正是动态检查要检验的内容。

@source E01 E04 E09''')
    p('07 四架僚机跟随的是什么','''僚机追踪的是随长机运动而更新的参考位置。前后距离按长机历史轨迹在水平面上的累计弧长生成，再向左右展开形成V形。两排的水平滞后距离分别为3米和6米，左右展开距离分别为2.5米和5米。

高度参考与长机同步。长机升高时，僚机也开始升高，而不是等长机的完整三维轨迹经过相同弧长后再升高。因此，这个参考不能解释为五架机始终保持一个刚性的空间V形。

跟踪输入包含位置误差项和速度误差项。位置误差把僚机拉向参考，速度误差帮助减小过冲。障碍附近还会调整参考的水平位置，减少参考点直接落入障碍的问题。

@eq follower

程序同时保留基础机间避让。它属于五机共同基础控制，关闭预测安全层时也不会被顺便关闭。所以安全层对照比较的是额外预测修正和制动检查的作用。

评估整队效果时，要看四架僚机逐时刻的动态跟踪误差、真实障碍净空和机间距离。最后整齐站好，不能代表飞行全过程队形都完全保持。

@source E01 E03 E04''')
    p('08 终点附近为什么要专门处理','''长机接近目标后，飞行任务从“沿路线前进”转为“减速并让整队归位”。原来的历史轨迹参考需要过渡到目标附近的固定V形参考。程序用2秒逐步混合两个参考，避免突然跳变。

这段时间长机停下来等待僚机，是设计中的终端归队。停滞统计会排除这种接近目标的预期等待，避免把正常结束动作错判成被障碍卡住。

参考完成过渡后，若僚机直接归位的路径受阻，程序可以为僚机分别规划末端局部路线。这些规划在控制循环中同步执行，其耗时需要计入对应周期。

Actor 也有终端退出规则。它帮助长机摆脱障碍后，要交回基础控制；到了编队终端保持阶段，不再继续请求临时航点。否则不断变化的局部目标会妨碍整队停稳。

成功判定仍用真实位置和速度检查五架机。噪声实验中，“测得已经到达”不足以让任务直接成功。这保证各组使用共同的评价标准。

@source E01 E03 E08''')
    p('09 当前 Actor 已经学好了什么','''Actor 可以理解为一个已经练过的局部方向选择器。输入是当前局部情况，输出是一个三维偏移方向。它在10月15日的离线训练中调整参数，本轮只读取保存的参数。

32个输入包含：3个目标相对位置分量，3个速度分量，以及26个空间方向上的测距值。目标相对位置按25米缩放，速度按3米/秒缩放，测距按12米量程归一化。这让不同单位的数据进入相近数量级。

@eq actor_observation

网络经过两个64单元隐藏层，输出3个[-1,1]范围的数。解码时先把三维动作投影到单位球内，再乘8米，形成距离当前位置最多8米的临时航点。三个方向共同受到8米长度约束。

@eq actor_waypoint

同一个 Actor 用于球体、圆柱、随机障碍和窗口等所有地图。地图改变的是输入，参数不变。任务之间重置的是轨迹、监督器状态和规划缓存，这些重置不等于把模型重新训练一次。

@source E01 E02 E03''')
    p('10 Actor 和 Critic 各负责什么','''离线训练中，Actor 负责提出动作，Critic 负责评价动作在当前状态下有多好。TD3 使用两个 Critic，借助回报和后续状态改进 Actor。训练结束后，飞行程序只需要 Actor 的前向计算。

训练回报奖励向目标取得进展和到达目标，惩罚耗时、频繁安全干预、过大动作、碰撞和持续停滞。它把“什么样的局部行为值得学习”转成训练信号。这个回报没有在本轮飞行中继续更新任何参数。

网络结构为32→64→64→3，隐藏层采用ReLU，输出采用tanh。参数数量按权重矩阵和偏置逐项计算：

@eq parameter_count

6467个单精度参数的纯数值存储为25868字节，约25.3KiB。模型文件还包含网络结构与元数据，所以文件大小略大。五架机没有各自复制、各自学习一套 Actor；在线仅长机在请求脱困时使用这套共享模型。

本轮加载检查确认模型文件只含 Actor 和元数据，Critic、训练代理和经验池都没有进入在线控制。原训练绑定的101份源码保存在独立参考目录中，供模型来源核对。

@source E02 E03 E10''')
    p('11 怎样判断该让 Actor 出场','''监督器每0.2秒查看一次长机最近3秒的运动。只有位置活动范围不超过0.05米、目标距离的变化不超过0.05米、最大速度不超过0.08米/秒，且还没有完成到达任务，才判为持续停滞。

Actor 介入后，至少保持2秒。随后只有在局部朝目标方向的通道具备足够净空、取得一定进展并维持前进状态时，才交回APF。退出后有3秒冷却期，减少两种模式反复切换。

一次介入最长30秒，最多介入3次。超出预算会进入制动并记录具体失败原因。预算失败没有被删除，也没有通过瞬移或重置速度掩盖。

一张普通地图可能全程不调用 Actor。因为全局路线和APF已让长机持续前进，停滞条件没有成立。日志把“模型已加载”和“网络实际调用次数”分别记录，不能看到模型存在就认为每一步都是神经网络在控制。

位置噪声也会进入监督器历史。若噪声本身的波动已经超过0.05米，真实无人机即使停住，测量轨迹也可能不满足停滞阈值。本轮保留原监督器阈值，通过实际记录检验它的行为。

@source E01 E03 E04 E09''')
    p('12 安全层怎样影响真正执行的输入','''安全层包含两部分。预测修正先用当前测量状态预测下一步，检查障碍、机间与边界风险，最多修正5轮。共同制动检查再判断本步执行之后，五架机是否仍有一条经过核验的制动轨迹。

候选动作不能通过时，优先执行已有的核验制动输入；测量变化使旧制动轨迹不再匹配时，重新从当前测量状态构造制动。若当前测量条件下已无法建立证书，程序执行物理限幅制动并记录证书不可用。

无论哪一种处理，真实系统都继续按指令和外扰推进。没有“安全层没算好就让位置原地不动”的分支。制动也通过加速度逐渐改变速度，而不是把速度直接设为零。

关闭安全层时，上述预测修正和共同制动检查一起关闭。共同的命令限幅、基础机间作用和真实碰撞检查仍保留。这样才能看出额外安全模块的增量作用。

噪声和外扰会让预测条件与真实运动产生差别。因此结果表始终使用真实扫掠距离评价安全，不能用“预测通过”代替“真实飞行没有突破安全距”。

@source E01 E03 E05 E09''')
    p('13 本轮怎样让四组对照公平','''所有比较都使用保存好的地图、起终点和初始状态。四个基本组只改变 Actor 和安全层两个开关，其他物理模型、控制增益、参考生成和完成条件保持一致。

'''+mdtable(['组别','Actor','安全层','主要用途'],[['A0S0','关闭','关闭','基础APF与编队'],['A1S0','开启','关闭','看Actor增量'],['A0S1','关闭','开启','看安全模块增量'],['A1S1','开启','开启','当前完整方案']])+'''

Actor 关闭组仍由同一套仿真器执行。开启组始终读取同一个冻结模型，不从前一张地图带来新的学习结果。噪声对照复用相同随机序列，外扰对照复用相同方向、幅值和绝对起始时间。

原来的R/S名称对应旧残差ADP实现，本轮使用清楚的双开关名称。旧结果保留在历史目录，本报告对当前控制器的判断全部来自这次重跑。

成功率按全部已声明任务计算；共同成功任务再配对比较时间、路径和队形误差。这样不会因为某组少完成几张难图，就让它看起来平均速度更快。

@source E03 E04 E06''')
    p('14 3840次运行分别在测什么','''本轮从已有输入目录恢复地图和初始状态，先冻结任务目录，再执行正式飞行。528条目录项包括不同实验中的复用地图，并不等于528张独立随机新地图。

'''+mdtable(['实验','条件或地图','运行次数'],[['安全层与Actor消融','154张 × 4组','616'],['APF动态检查','50种 × 4组','200'],['噪声与外扰','12图 × 38条件 × 4组','1824'],['五机接入','128图 × 2组','256'],['规划与脱困六组合','144图 × 6组','864'],['单机原留出集','40图 × 2组','80'],['合计','已声明任务全部保留','3840']])+'''

安全层地图包含原随机、困难和核对场景。动态检查包含正对球体、微小偏移、圆柱顶部和正常规划场景。抗扰实验包含无扰动、位置噪声、加速度脉冲和联合条件。

五机接入和六组合实验检查 Actor 在整队系统中的增量效果。单机任务单独统计，用于检查原脱困能力，不能把单机成功直接当成五机成功。

历史二维25000次扫描与未知地图开发没有混入这3840次运行。

@source E03 E04 E10''')
    p('15 原安全层地图得到什么结果',f'''四组在同一154张地图上的结果如下。最小距离覆盖全部已经执行的运动，包括失败任务；不是只从成功轨迹里挑出的安全距离。

{four_table('repair')}

{pair_text(paired('repair'))}

若安全层提高成功次数，同时增加时间或路径，这表示它为了保留制动能力，执行了更保守的动作。判断收益时要把避免失败与这些代价一起看。

Actor 的增量可以在相同安全开关下比较。例如A0S1与A1S1使用同一个安全模块，差别来自冻结 Actor 与监督器。若两组轨迹相同且调用次数为0，说明这些地图没有请求学习策略接管。

@source E04 E06 E09''')
    p('16 正对球体和圆柱顶部会怎样',f'''动态检查共50种条件，包括38种直接压力条件和12种正常规划条件。球体场景既有严格正对，也有左右、上下的偏移；偏移幅度包括1e-7米、1e-4米和0.25米。初速度同时覆盖静止和3米/秒。

圆柱场景让长机从柱顶附近不同高度接近，还包括斜向、擦边和正常规划通过。五架机共同运动，碰撞检查覆盖每一架，不能只盯住长机。

{four_table('dynamic')}

完整A1S1组有 {stat(sel('dynamic','actor',True))['stall']} 次任务出现远离终点的持续低速段，有 {stat(sel('dynamic','actor',True))['sway']} 次满足重复侧向摇摆判据。这两种现象分别判断，不把控制指令反号直接等同于轨迹来回摆动。

原始MAT保存位置、速度、命令、真实总加速度、最近障碍距离、退化标记、安全触发和Actor模式。异常可以按任务ID直接定位到曲线与时刻。

@source E04 E09 E12''')
    p('17 噪声实验究竟加了什么',f'''位置噪声加入五架机的三个位置反馈轴，每步更新一次。每轴标准差分别为0.02、0.05、0.10米，每个非零等级使用3条固定随机序列。0.10米表示每轴误差的标准差，并不表示误差永远小于0.10米。

真实位置用于物理推进和碰撞判定，带噪位置用于基础控制、编队参考、安全计算和监督器历史。速度反馈在这套协议中保持精确，障碍几何也保持精确。

'''+mdtable(['每轴标准差 m','A0S0','A1S0','A0S1','A1S1'],[[str(n)]+[ratio(sel('robust',m,s,kind='noise',noiseStd=n)) for m,s in [('none',False),('actor',False),('none',True),('actor',True)]] for n in [.02,.05,.1]])+f'''

每格为36次任务。纯噪声A1S1组共有 {stat(sel('robust','actor',True,kind='noise'))['active']} 次任务实际请求Actor。抗噪结果首先反映当前整套反馈系统，Actor是否参与必须再看调用记录。

较大的位置噪声会使参考点和控制命令更频繁变化。即使任务最终成功，也要看队形误差、安全净空和真实轨迹是否出现明显摇摆。

@source E03 E04 E09''')
    p('18 外部加速度在什么时候施加','''加速度脉冲沿±Y、±Z四个世界坐标方向施加，幅值为0.10、0.30、0.50、1.00米/秒平方，保持2秒。五架机同时受到同向加速度，不把它伪装成一条控制指令。

本轮复用原实验已经固定的绝对起始步。原起始步来自无噪声参考飞行首次接近障碍表面6米的时刻；重跑所有开关和幅值仍使用同一时间，避免哪组飞得快就晚给它加干扰。结果中另报真正施加时的距离。

'''+mdtable(['脉冲幅值 m/s²','A0S0','A1S0','A0S1','A1S1'],[[str(n)]+[ratio(sel('robust',m,s,kind='gust',gustAmplitude=n)) for m,s in [('none',False),('actor',False),('none',True),('actor',True)]] for n in [.1,.3,.5,1.]])+'''

每格为48次任务。指令限幅后再叠加脉冲，所以最大总加速度可以高于3。程序没有再次把总加速度裁回3，从而保留了真实的外扰作用。

脉冲结束后继续观察至少5秒。到达条件在观察时限前出现，也不会立即终止实验，防止漏掉干扰后的恢复过程。

@source E03 E04 E07''')
    p('19 怎样判断干扰后恢复了',f'''“最后成功到达”和“干扰后很快恢复”不是同一个指标。本轮把每次受扰飞行与同地图、同Actor开关、同安全开关、同位置噪声序列的无脉冲飞行对齐。

恢复要求五架机与对应无脉冲轨迹的最大位置差不超过0.5米，最大速度差不超过0.25米/秒，并连续保持1秒。只有在脉冲结束后的5秒窗口内确认，才计入快速恢复。

没有完整对照、脉冲结束前已经失败、或没有完成观察窗口的任务，都不会被算成恢复成功。恢复表给出具体原因，读者可以看到“未恢复”来自误差仍大，还是飞行已失败。

联合条件固定为每轴0.02米位置噪声与0.30米/秒平方的2秒脉冲，覆盖4个方向、3条噪声序列。它用于检查两种影响叠加后的表现，没有把所有强度随意组合。

联合条件当前完整组成功 {ratio(sel('robust','actor',True,kind='joint'))}。原始恢复时间与对照任务ID保存在恢复表中。报告讨论的是这些已经测试的幅值和方向，不把“全部成功”扩展成所有风场都能抵抗。

@source E03 E04 E07''')
    p('20 指令抖动与轨迹摇摆怎样区分','''位置噪声会立即影响反馈误差，而位置运动还要经过速度积分。因此，命令快速正负变化，真实轨迹未必出现同样幅度的来回摆动。把这两种曲线放在一起，才能判断现象发生在哪一层。

@figure 噪声控制与轨迹 | 图2 同一正式噪声任务的长机侧向命令 速度及真实安全距离

本轮分别记录显著命令分量反号和重复侧向轨迹摇摆。轨迹摇摆判据要求5秒内至少4次有效侧向速度反号，速度幅值达到0.15米/秒，侧向位置跨度至少0.30米。普通绕障时的一次转弯不会自动满足这个判据。

加速度极值也单独统计。3米/秒平方约为0.31g的平动加速度量级。这个质点模型没有姿态和机体载荷方程，图中数据直接说明的是指令与真实总加速度，不能把它换成未计算的结构过载。

@source E04 E09 E12''')
    p('21 五机接入能否保持原来正常飞行',f'''五机接入集包含12种固定场景、48张普通随机地图、24张结构化地图、40张局部窗口压力地图和4张全封堵局部反例，共128条输入。对照组与Actor组都开启同一套安全模块。

基础组成功 {ratio(sel('integration','none',True))}，Actor组成功 {ratio(sel('integration','actor',True))}。Actor在 {stat(sel('integration','actor',True))['active']} 次任务中实际出场。

{pair_text(paired('integration','none_S1','actor_S1'))}

@figure 同图结果 | 图3 同一安全模块下的各实验配对完成情况

全封堵场景用于检查系统如何拒绝或停止，失败是保留的反例。汇总分母没有先删掉这些地图；家族表再把可通与封堵条件分开解释。

@source E03 E04 E06''')
    p('22 同一 Actor 怎样让整队脱困','''窗口压力场景让局部APF面对一个几何上可通过、但当前局部力可能相互抵消的通道。它明确关闭长机全局路线，目的是检验局部脱困动作。

@figure Actor脱困时序 | 图4 同一正式窗口任务的五机高度和加速度指令

长机持续停滞后，Actor根据目标、速度和26束距离提出局部航点。长机开始改变空间运动，僚机在高度同步、历史滞后参考和障碍作用下跟随。局部通道和前进条件满足后，监督器退出，让APF继续接近原目标。

这张图不能单独证明Actor普遍优于其他方法。它说明了一次完整的输入、介入、整队响应和退出过程，数值评价仍使用同图批量对照。

如果长机已经绕开而僚机跟不上，整队仍可能超时；若任一架机突破安全距，任务仍判失败。脱困成功必须落实到五机共同完成条件。

@source E04 E09''')
    p('23 全局规划和 Actor 各贡献多少',f'''六组合实验在同一144张地图上，同时比较全局规划开启或关闭，以及无脱困、规则脱困、Actor脱困三种方法。三种方法共享监督器介入和退出条件，也共享基础控制与安全模块。

@figure 六组合结果 | 图5 全局规划与局部脱困策略的六组合对照

{conclusion()}

规则方法使用同样的局部26束距离和目标方向提出动作，没有拿到预先标出的通道答案。它是判断“学习是否比一个直接规则更值得”的对照。Actor相对纯APF有收益，并不自动意味着它优于规则。

全局规划本身能够绕过很多局部停滞位置。这种条件下 Actor 的接管次数可能较少，甚至为0。评价它时要同时看正常飞行是否保持，以及卡住后的增量成功次数。

@source E03 E04 E06''')
    p('24 单机结果应该怎样使用',f'''单机原留出集由16张接近窗口场景和24张普通场景组成。它使用一个无人机的局部控制与安全检查，完成条件是到达目标并减速；不需要等待四架僚机归队。

本轮重新运行同一个冻结模型，基础APF成功 {ratio(sel('single','none',True))}，Actor方案成功 {ratio(sel('single','actor',True))}。这组结果直接检查原有脱困能力是否还在。

{pair_text(paired('single','none_S1','actor_S1'))}

单机能绕过障碍，说明局部策略能产生有效空间运动。五机还需要检查僚机参考、机间距离和终端归位，所以报告另列128图接入和144图六组合。

这些地图以前已经用于留出验证；本轮是同一输入的重跑，不新增独立随机样本数。模型也没有借这次运行再次调参数。

@source E03 E04 E06''')
    p('25 物理步长与计算耗时','''仿真步长规定物理时间怎样前进，CPU耗时规定程序实际算了多久。一次0.05秒的物理步，如果需要0.08秒计算，就已经超过50毫秒预算。

本轮在并发实验停止后，单独串行重跑五机无障碍、球体、窗口、密集地图和Actor局部脱困场景，每种正式测3次，完整预热另存。每个周期从状态反馈到计算命令、推进和记录状态一起计时，周期中的重规划也算进去。

初始路线规划和预建图不在持续控制循环内，分别列出。绘图和磁盘保存不计入周期。报告给出均值、P95、P99、最大值，以及超过50毫秒的次数；这些数不能相互替代。

P99低于50毫秒，表示约99%的观测周期较短，不表示最大值也小于50毫秒。若少数末端规划周期很长，需要直接看到那次事件，不能把它从计时数组里移走。

二维参考也在同一机器和单线程设置下另测，用来说明计算规模变化。它属于历史二维对照，不混入统一Actor的3840次效果统计。

@source E08''')
    p('26 怎样亲自复现一张地图','''在工程根目录启动MATLAB，执行main，默认进入已知地图统一Actor控制器。需要换图时，先调用default_params3d，再修改sceneId或seed，最后把参数传给main3d。

main3d返回的result里，model给出控制器版本、模型和参数指纹、是否加载Actor；controller给出实际调用次数、介入次数和模式切换事件；history保存逐步状态与控制。

参数中的adp.wA和adp.wC属于已关闭的旧残差接口。当前Actor的权重从controller_assets读取，开启方式由controller.method决定；核对新Actor应读取result.model和result.controller。

若要复现完整实验，添加experiments3d/unified_actor_20261029路径，执行run_all_unified。提交的catalogue.mat包含本轮地图和任务目录，复现不依赖本机之前的旧实验目录。

看一条失败记录时，可以先查逐任务表中的任务ID、失败原因和最小距离，再打开同名MAT。把真实位置、速度、命令、外扰、Actor模式和安全触发放到同一时间轴，就能定位是何时卡住、谁触发制动、哪架机首先突破安全距。

新文件、检查点和交付物均保存在F盘。GitHub主分支已包含当前源码和唯一模型，完整原始飞行记录保存在本地证据目录，公开的结果表与报告提供对应索引。

@source E01 E04 E09 E10 E11''')
    p('27 读完后应当形成的判断',f'''当前系统的工作分工已经明确：全局规划找路线，APF和编队控制负责连续跟踪，冻结Actor负责需要时的局部脱困，安全模块限制候选动作，真实模型与独立检查决定最终结果。

{conclusion()}

判断控制器好坏要看同一条件下的增量。普通地图没有接管，说明Actor没有参与那次任务；局部压力场景由失败变成功，才是直接的脱困收益。与规则对照的差别则说明这种收益是否必须依赖学习。

安全层的收益和计算代价、位置噪声造成的控制变化、真实轨迹是否摇摆、外扰后的恢复速度，也都有各自的数据。报告没有用一条漂亮轨迹替代整组结果。

这套程序已经能把五机运动、模块切换和失败原因连成可追溯的记录。下一轮修改应针对记录中出现的具体失败，把同一输入前后对照，而不是继续把不同控制版本的结果混在一起。

@source E04 E06 E07 E08 E10''')
    p('28 证据索引','''下面的链接对应本版源码、协议、逐任务结果和完整轨迹。正文中的E编号均可在这里找到。

@evidence''')
    starts={1,3,9,13,15,17,21,25,26,28}
    return ''.join(('\n\n@page\n\n' if i in starts else '\n\n' if i else '')+v for i,v in enumerate(pages))

def timing_table(include_2d=False):
    folder=RUN/'serial_timing';names=['S0_empty','S1_sphere','S8_window','normal_61036','window_63001']
    if include_2d:names+=['legacy_2d_obs0','legacy_2d_obs15']
    labels={'S0_empty':'三维无障碍','S1_sphere':'三维单球','S8_window':'三维全局窗口','normal_61036':'三维密集地图','window_63001':'三维Actor脱困','legacy_2d_obs0':'历史二维无障碍','legacy_2d_obs15':'历史二维15障碍'}
    data=[]
    for name in names:
        a=np.concatenate([np.loadtxt(folder/f'{name}_r{k}_cycle_ms.csv',delimiter=',',ndmin=1) for k in [1,2,3]])
        data.append([labels[name],fmt(float(a.mean()),3),fmt(float(np.percentile(a,95)),3),fmt(float(np.percentile(a,99)),3),fmt(float(a.max()),3),f'{np.sum(a>50)}/{len(a)}'])
    return mdtable(['场景','均值 ms','P95 ms','P99 ms','最大 ms','超50ms/周期'],data)

def teacher_text():
    pages=[]
    def p(title,body):pages.append(('## '+title+'\n\n' if title else '')+body.strip())
    p('',f'''# 无人机编队智能避障项目技术报告

教师审阅版 统一 Actor 全量重验

版本 2.3.4-unified-actor-20261029  整理日期 2026年10月30日

本项目实现已知三维静态障碍地图中的五机规划、避障、编队跟踪和终端归队。当前统一控制器采用分层全局规划、APF及机间作用、冻结Actor局部航点监督器，以及预测修正与共同制动检查。在线不加载Critic，不更新学习参数。

本轮将所有已知地图默认入口统一到10月15日训练得到的同一个6467参数Actor，并重跑3840次已声明任务。训练模型、原训练绑定源码和完整地图目录随代码提交；逐任务保存模型指纹、实际调用次数、模块开关、真实状态和失败原因。

{conclusion()}

报告按实际实现和本轮重验结果展开。历史残差ADP、二维扫描与未知地图开发的结果保持独立。本报告不再用旧残差实现的安全层或扰动统计评价当前冻结Actor。

@source E01 E02 E03 E04 E10 E11''')
    p('01 任务定义与完成条件','''系统由1架长机和4架僚机构成。环境给定球体、有限高度垂直圆柱与箱体的三维几何。无人机采用加速度控制的二阶质点模型，各机同步更新；工作空间为[0,80]×[0,80]×[0,30]米。

五机成功条件为：长机到目标距离不超过1米，四架僚机相对动态参考的最大误差不超过1米，五机最大速度不超过0.25米/秒，参考未阻塞，并连续满足1秒。所有条件用真实运动状态进行评价。

障碍硬安全距为0.6米，机间硬安全距为1米。逐周期检查每架机的扫掠运动与同步机间最近距离。碰撞结果采用源码中的硬安全距离判定；本轮另记障碍表面距离不大于0的表面接触。

无障碍、单球、有限圆柱、上绕、下绕、窗口与末端归队共同构成基础检查，随机与困难地图再覆盖复杂几何。全封堵反例保留在总体分母中，并在家族分析中单列。

本版面向已知地图，测距是基于真实障碍几何的局部射线查询。位置噪声实验不改变地图几何，速度反馈保持精确。

@source E01 E03 E04''')
    p('02 控制信息流及函数接口','''main和main3d均进入run_simulation3d，后者显式选择统一冻结Actor控制器。各地图只改变几何、种子或任务参数，不选择另一个模型。Actor关闭及规则脱困仅作为明确的实验组。

@figure 控制信息流 | 图1 与当前源码对应的已知地图信息流

'''+mdtable(['模块','主要函数','输入与输出'],[['统一入口','run_simulation3d','参数与地图 → 统一仿真结果'],['模型核验','ua_load_actor','固定模型 → Actor 配置 指纹'],['全局规划','plan_layers3d','地图 起终点 → 三维航点'],['编队参考','formation_reference3d','测量状态 历史 → 逐机参考'],['局部控制','f5_control','航点 参考 障碍 → 基础加速度'],['监督器','fc_decide','测量历史 局部观测 → 临时航点与模式'],['安全选令','resolve_safety3d ua_select_command','候选输入 测量状态 → 执行命令'],['物理与评价','motion_step3d check_swept3d','真值 命令 外扰 → 下一真值与安全距离']])+'''

APF、编队跟踪与机间作用在基础输入中共同组成，不机械画成三个串联控制器。Actor的在线作用是调整长机临时航点，最终加速度仍经过共同控制和安全处理。

@source E01 E03''')
    p('03 离散动力学与限幅顺序','''每个物理步使用0.05秒半隐式Euler更新。最终命令先按3米/秒平方的模长上限裁切，外部加速度随后叠加，更新后速度再按3米/秒的模长上限裁切，最后以新速度推进位置。

@eq saturation

@eq motion_student

sat的下标v表示速度模长上限，u表示安全处理后的最终有界命令，d表示未再次裁切的外部加速度。由此，命令模长上限与总实际加速度上限不混用。

所有无人机使用同一旧状态同步推进。独立Python复核从保存的初态、命令、外扰和步长重新计算每次位置与速度，并以1e-10容差检查；同时核对记录的真实总加速度、测量噪声入口和速度上限。

本轮试运行还将无噪声普通场景和Actor实际介入场景，与原冻结Actor五机实现逐点比较，保存的状态和命令完全一致。噪声脉冲试运行独立重构误差为0。

@source E01 E05 E10''')
    p('04 三维几何与连续运动检查','''球体净空采用位置到球心距离减半径。有限圆柱使用径向与高度联合的有符号距离，覆盖侧面、顶底面和棱边切换；箱体使用三维有符号距离。

@eq sphere

评价器检查每个物理步的完整位置线段。两机之间先形成同步相对线段，再计算该相对线段到原点的最小距离，从而覆盖两机同时交叉的情况。

预测安全层和实际物理模型使用相同的命令限幅、积分和速度限幅顺序。共同制动检查还验证候选一步后的制动轨迹，避免只确认当前一步不碰撞，却已经失去后续制动空间。

制动的障碍线段证书采用有界细分，最大深度12，几何容差1e-6；机间制动按每个同步时间区间检验。该核验针对名义模型及其输入状态，受噪状态下的真实安全仍由独立扫掠结果判断。

距离统计覆盖所有执行步骤，包括碰撞前最后一步、超时和预算终止。没有运行历史的规划或生成失败另列，不填造最小飞行净空。

@source E03 E04 E05 E09''')
    p('05 有限高度层与规划规模','''基础层为5、10、15、20、25米，水平网格分辨率1米。同层八邻接，邻层仅垂直连接；边代价按三维长度并叠加累计升降权重。每条边、连接起终点的接入段与简化长线段均检测净空。

@eq route

当前路径规划净空为1.2米，动态参考调整净空为4米，障碍硬安全距为0.6米，三者用途不同。任务书中的建议参数没有直接作为本轮实际参数引用。

固定层搜索失败后，程序从障碍投影自由高度区间选择有界补层，最多新增8层。工作高度层局限于当前地图和当前规划调用，不会在线改变其他地图的高度配置。

81×81单层理论容量为6561，五层为32805。与81³=531441的纯算术对照中，五层容量为约6.17%；但z方向空间范围与分辨率不同，实际可用节点还受障碍和边界影响，不能把容量比例当作实测速度比例。

规划预建图耗时和存储单列。同步在线重规划、Actor退出后的接回全局路线和末端僚机规划，均保留事件耗时及对应完整控制周期。

@source E01 E03 E08 E09''')
    p('06 APF结构与切向退化规则','''基础控制将跟踪吸引、障碍法向与切向作用、机间避让共同组合。障碍风险提高时增加障碍作用权重。长机使用当前航点或目标保持控制，僚机使用动态参考位置与参考速度反馈。

@eq base

切向方向由当前航点方向在障碍切平面内的投影得到。障碍距离小于作用范围一半且目标距离足够大时，程序启用切向项。

@eq tangent

投影模长小于1e-6时，先用原始目标方向再次投影；仍小于阈值则记退化，并在基础控制中附加减速。程序没有用任意世界轴旋转来填入一个不连续方向。

全局路线条件下保留停滞重规划；监督器另外检查持续停滞并请求统一Actor。退化标记、安全层触发和Actor介入分别保存，三者不能互相代替。

本轮将静止与3米/秒初速度、严格正对和极小偏移、柱顶附近不同高度等条件纳入动态运行，结果见第14节。

@source E01 E03 E04''')
    p('07 编队参考与终端切换','''水平弧长滞后按长机水平速度积分生成，前后排参考分别滞后3米和6米，并横向展开2.5米和5米。高度与长机同步。低水平速度时保留最近有效航向，避免朝向由近零速度决定。

@eq follower

障碍附近可以投影调整参考的水平位置，并进一步检查参考点之间的间距。此策略为水平动态参考和同步升降，不是沿完整三维轨迹的固定弧长保持，也没有主动压缩因子。

末端保持以2秒混合参考过渡到固定V形。需要时为僚机生成局部归位路线；这些事件规划计入实际在线周期。Actor在设计的终端归队阶段停止提出临时航点。

队形RMS按每个时刻四架僚机动态参考误差的最大值，再计算整个任务的时间均方根。路径和队形指标报告成功样本数；跨组比较另列共同成功集合。

@eq rms

@source E01 E03 E04 E06''')
    p('08 冻结Actor的输入动作与共享方式','''在线策略为32→64→64→3的前馈网络，两层隐藏层用ReLU，输出用tanh。输入由目标相对位置3维、速度3维和26束空间距离组成，分别按25米、3米/秒和12米量程归一化。

@eq actor_observation

输出动作先投影到单位球，再乘8米，形成相对当前位置的三维临时航点。动作不是直接加速度；临时航点仍须经过共同基础控制与安全模块。

@eq actor_waypoint

6467个固定参数在所有地图共享，飞行期间不更新。每次任务新建监督器状态、轨迹和规划缓存；不会把某图在线学习后的参数传到下一图，也不会在每张地图重新初始化随机Actor。

加载器同时核验模型文件指纹、参数字节指纹、原训练绑定源码和当前推理接口。原训练参考目录只用于绑定审计，不加入执行路径；当前运行源码另有独立指纹。

所有正式任务的模型文件SHA256均为51535c86771cd8e7dd189d7080f43010edbbbe8e0995ad6065391ae2f03e5740。

@source E01 E02 E03 E10''')
    p('09 离线训练与在线推理的职责','''原Actor由TD3离线训练，使用两个独立Critic、延迟Actor更新、目标网络和平滑目标动作。训练回报包含目标进展、耗时、安全干预和动作惩罚，终端到达加150，碰撞减200，超时减25，持续停滞减15。

在线部署只导出Actor与元数据。6467参数对应25868字节单精度数值。两个Critic每个为6529参数，属于离线训练规模，在线未加载。训练状态、目标网络、优化器和经验池不计入当前飞行控制器的可调参数。

@eq parameter_count

原配置中的训练上限和实际导出训练轮数在模型来源记录中分别核对，本轮没有重新训练、选择地图专用模型或依据新结果调学习率。

同一个Actor在多张地图上得到不同输出，来自输入状态和测距不同。它不需要重新学习就可以产生不同动作。任务之间保持参数不变，是本轮同图消融成立的前提。

当前Actor输出临时航点，与历史2.3.4残差Actor输出小幅加速度修正的机制不同。本报告的参数规模、时序和收益均按当前实现定义。

@source E02 E03 E10''')
    p('10 介入退出与失效处置','''监督器每4个物理步执行一次，即0.2秒。持续3秒内位置活动范围不超过0.05米、目标距离进展绝对值不超过0.05米、最大速度不超过0.08米/秒且未到达，触发局部脱困。

Actor模式至少保持2秒。局部目标通道满足额外净空、至少取得0.5米目标进展、预测前向速度达到0.05米/秒，并连续清晰1秒后，可以退出到3秒冷却。靠近目标时另有终端通道交接。

每次接管最长30秒，最多3次；超限记录escape_timeout或activation_limit并执行制动。全局规划开启时，Actor退出还需要重新接回全局目标路线，接回失败也单独记录。

纯基础组与规则组共享这些介入退出条件。规则组使用同一局部观测，不获得场景类别、窗口答案或未来轨迹。

模型已加载、Actor实际调用、激活次数和模式事件分别记录。Actor调用为0的任务不归因于策略输出。测量噪声进入监督器历史后可能影响停滞判定，本轮保留该效应。

@source E01 E03 E04 E09''')
    p('11 安全修正与真实物理推进','''安全开启组使用最多5轮的预测修正，加上共同制动可行性核验。候选下一状态既要通过本步几何检查，也要保留五机共同减速至停的名义可行轨迹。

当候选不满足条件，优先取此前核验制动轨迹的第一条命令；若测量状态与旧轨迹不匹配，则从当前测量状态重建制动。当前证书不可得时，仍执行有界最大制动并记录不可得状态。

物理推进与命令选择分开。真实系统始终执行motion_step3d；拒绝候选不会冻结位置、瞬间清零速度或移除外扰。外部加速度在最终命令限幅后叠加。

安全关闭组同时关闭预测修正和共同制动守卫，保留共同模长限幅、基础机间作用和独立真实扫掠检查。没有在名义关闭组暗中保留同一个安全模块。

零噪声名义情况下，可用证书与实际制动轨迹一致；噪声、外扰条件下的真实执行仍逐段检查。原始记录保存证书状态、安全修正、制动介入和真实最小距离。

@source E01 E03 E05 E09''')
    p('12 实验目录分组与统计规则','''正式运行前冻结528条输入目录项和3840条任务。不同实验可能复用相同几何，目录项数量不作为独立随机样本数量。历史二维与未知地图结果不进入本轮总数。

'''+mdtable(['实验族','固定输入','运行数'],[['安全层与Actor消融','原154图 四开关组合','616'],['APF动态检查','50种固定条件 四开关组合','200'],['抗扰','12图 38条件 四开关组合','1824'],['五机接入','128图 基础与Actor','256'],['规划与脱困因子对照','144图 两路线策略与三脱困策略','864'],['单机原留出诊断','40图 基础与Actor','80']])+'''

四组记为A0S0、A1S0、A0S1、A1S1，A1开启同一冻结Actor，S1同时开启预测修正和共同制动。各组重置仿真与监督器状态，共享模型参数和完成规则。

保留生成、规划、碰撞、越界、超时和预算终止结果，不按控制器表现筛选地图。成功样本平均值单独注明分母，共同成功集合再报告逐图差值，避免成功集合变化形成选择偏差。

@source E03 E04 E06''')
    p('13 原154图消融结果与配对代价',f'''四组结果如下。随机、困难和核对场景的分层统计随逐任务表保存。

{four_table('repair')}

A1S0与A1S1用于观察安全模块在同一Actor方案上的增量。{pair_text(paired('repair'))}

A0S1与A1S1用于观察同一安全条件下Actor增量。{pair_text(paired('repair','none_S1','actor_S1'))}

安全距离覆盖全部执行样本。对于发生硬安全距突破的任务，失败类型与最小距离共同保留，未用成功轨迹的较大净空替换。

@source E04 E06 E09''')
    p('14 APF退化动态复测结果',f'''50种条件覆盖球体严格正对、左右和上下偏移，偏移量1e-7、1e-4和0.25米；初速度为0或3米/秒。圆柱压力条件覆盖柱顶高度附近水平接近、斜向接近和顶部擦边，另有12种正常规划条件。

{four_table('dynamic')}

完整组A1S1中，远离终点的持续低速任务有 {stat(sel('dynamic','actor',True))['stall']} 次，满足重复侧向摇摆判据的任务有 {stat(sel('dynamic','actor',True))['sway']} 次。所有显著命令反号另外记录，未将命令抖动等同于位置振荡。

最大指令模长为 {fmt(stat(sel('dynamic','actor',True))['maxCommand'],6)} 米/秒平方，最大真实总加速度为 {fmt(stat(sel('dynamic','actor',True))['maxAcceleration'],6)} 米/秒平方。此组无外扰，命令与实际加速度按同一物理模型核对。

现象判据、逐任务条目、对应轨迹与关键时刻另附可核查记录。原始位置、速度、控制、安全标记和退化标记保存在同名MAT中。

@source E04 E09 E12''')
    p('15 位置噪声协议与分级结果','''位置噪声为三轴独立零均值高斯反馈误差，每个物理步更新；非零每轴标准差为0.02、0.05、0.10米，使用3条固定随机序列。各开关组复用同一噪声样本，速度反馈保持精确。

真实状态只用于物理推进及评价；控制、动态参考、安全检查和监督器历史使用测量状态。初始任务位置和已知地图由固定协议给定。随机序列与逐步噪声已保存。

'''+mdtable(['标准差 m','A0S0','A1S0','A0S1','A1S1'],[[str(n)]+[ratio(sel('robust',m,s,kind='noise',noiseStd=n)) for m,s in [('none',False),('actor',False),('none',True),('actor',True)]] for n in [.02,.05,.1]])+'''

@figure 噪声等级 | 图2 位置噪声分级完成比例 每格36次配对任务

每轴标准差不是误差硬上界，0.10米也不代表三维位置误差最大为0.10米。等级评价同时包含真实净空、队形RMS、命令变化和Actor调用情况。

@source E03 E04 E09''')
    p('16 外部加速度与联合干扰结果','''输入外扰为0.10、0.30、0.50、1.00米/秒平方的恒幅2秒脉冲，沿±Y、±Z方向同时作用于五架机，在最终命令限幅后叠加。零外扰基线与每个受扰组使用相同地图、时间轴和噪声序列。

脉冲起始步沿用原预先固定记录，源于无噪声参考整队首次进入6米障碍表面距离的时刻。为保持严格的绝对时间对照，本轮不按各组实时距离重新选时，另记录实际施加时的真实净空。

'''+mdtable(['幅值 m/s²','A0S0','A1S0','A0S1','A1S1'],[[str(n)]+[ratio(sel('robust',m,s,kind='gust',gustAmplitude=n)) for m,s in [('none',False),('actor',False),('none',True),('actor',True)]] for n in [.1,.3,.5,1.]])+f'''

每格48次。联合条件为0.02米位置噪声与0.30米/秒平方脉冲，4方向、3序列；A1S1成功 {ratio(sel('robust','actor',True,kind='joint'))}。

脉冲结束后至少观察5秒。真实总加速度、速度限幅及真实安全距离全部保存，没有把扰动后的总加速度再次限制为3来削弱实际注入。

@source E03 E04 E07 E09''')
    p('17 恢复能力与控制波动解释','''恢复指标使用同地图、同方法、同安全开关、同噪声序列的无脉冲运行作参考。脉冲结束后5秒内，五机最大位置差≤0.5米且最大速度差≤0.25米/秒，连续保持1秒才计为恢复，报告确认时刻。

对照缺失、脉冲结束前失败或观察不足，均不计恢复。完成任务和快速恢复分别统计，避免把最终成功替代瞬态性能。

@figure 噪声控制与轨迹 | 图3 同一0.10米位置噪声任务的命令 速度和真实距离

位置噪声首先放大高频反馈变化，速度与位置还经过积分。命令反号可能先于明显轨迹摇摆出现。报告使用独立的侧向速度和位置幅度判据，再与原始时序交叉检查。

当前模型仅计算平动状态。最大实际加速度除以9.80665用于说明平动g量级；没有将这一比值命名为机体结构过载。

@source E04 E07 E09 E12''')
    p('18 五机接入及完整脱困过程',f'''128条输入包含12固定、48普通随机、24结构化、40局部窗口和4全封堵局部反例。基础组与Actor组共享安全模块和五机完成条件。

基础组成功 {ratio(sel('integration','none',True))}，Actor组成功 {ratio(sel('integration','actor',True))}。{pair_text(paired('integration','none_S1','actor_S1'))}

@figure Actor脱困时序 | 图4 正式窗口任务的五机高度和执行命令

该案例展示长机请求局部航点、整队产生高度运动、满足退出条件并继续接近原目标的过程。原始模式事件与安全标记可按同一任务ID读取。

五机成功不是单机逃出窗口。任一僚机碰撞、无法跟上参考或不能在终端归队，均由共同完成条件反映。

@source E03 E04 E06 E09''')
    p('19 规划与三类脱困策略因子对照',f'''144张地图包括普通48、结构化24、单窗口32、双窗口24、凹形8和全封堵8。每图执行全局规划开启或关闭，与无脱困、规则脱困、Actor脱困组成的6个组合。

@figure 六组合结果 | 图5 同图六组合结果

{conclusion()}

规则方法使用同样的局部32维观测与监督器条件，以测距、目标方向和方向滞后提出临时航点，不使用已标注通道或全局搜索。它检验学习策略相对直接规则的额外价值。

在全局路线能保持前进时，Actor可能从未介入。学习收益应结合实际接管、同图由失败变成功和共同成功代价判断，不能直接把完整方案成功全部归因于Actor。

@source E03 E04 E06''')
    p('20 单机诊断与五机证据的关系',f'''40条单机原留出输入由16张接近窗口和24张普通场景组成。基础APF成功 {ratio(sel('single','none',True))}，Actor成功 {ratio(sel('single','actor',True))}。

{pair_text(paired('single','none_S1','actor_S1'))}

单机完成条件为到达并减速，与五机动态参考最大误差和终端保持条件不同。因此单机结果单列，不与五机任务的成功率或轨迹代价混合。

本轮沿用此前留出输入，是对原控制能力的重跑检查，不新增独立留出地图。全图共享Actor，没有按本轮结果再次选择训练检查点。

@source E03 E04 E06''')
    p('21 串行完整周期耗时','''正式并发批次全部停止后，另用单计算线程运行五机无障碍、单球、全局窗口、37障碍密集地图和局部窗口Actor场景。每场景先完整预热，再正式测3次。表中将3次逐周期原数组合并统计。

'''+timing_table(True)+'''

周期计时覆盖反馈、监督器、基础控制、安全选令、真实推进、评价和循环内日志赋值；同步重规划及末端局部规划发生时计入该周期。图形与磁盘写入在周期外。初始规划及预建图另外保存，不隐去冷启动成本。

P95、P99和最大值从同一组完整周期计算，没有相加模块分位数。二维行为使用历史二维控制器，仅作为同硬件计算参考，不混入当前Actor效果统计。

软件环境、可见硬件、线程设置、原始周期数组、初始规划、重规划和末端规划事件均随计时目录交付。并发3840次批量吞吐量不作实时延迟证据。

@source E08''')
    p('22 参数规模与计算预算','''当前在线Actor只有6467个共享可调标量，输入32、输出3。纯单精度参数为约25.3KiB；模型文件还含网络结构和来源元数据。五机状态、规划缓存和轨迹日志不计入学习参数。

'''+mdtable(['项目','实际配置','核对来源'],[['物理循环','0.05秒 20Hz','default_params3d'],['监督器决策','每4步 0.20秒','configuration.mat'],['Actor','32→64→64→3 6467参数','Learnables逐项计数'],['共享方式','长机调用 同一模型跨图共享','ua_load_actor ua_simulate'],['基础规划','5层 32805理论节点容量','plan_fixed_layers3d'],['适应补层','最多新增8层','adaptive_height_layers3d'],['预测修正','最多5轮','resolve_safety3d'],['几何证书','细分深度12 容差1e-6','cfg.safety'],['退出与失败预算','介入最长30秒 最多3次','apf_adp_config']])+'''

网络数值规模较小，不代表整条控制周期必然满足50毫秒。当前计算量还受到地图几何查询、制动轨迹长度和同步事件规划影响。轻量性应同时引用参数表和第21节的完整周期实测。

@source E02 E03 E08''')
    p('23 结果判断及尚存问题',f'''{conclusion()}

本轮的直接判断是：冻结Actor是否改善局部停滞，应看同图失败转成功和实际接管记录；是否优于规则，应看六组合的相同局部信息对照；是否维持正常飞行，应看原本成功地图的配对轨迹和终端条件。

位置噪声与外部加速度的结果用于评价当前整套闭环。只有实际发生Actor调用的受扰任务，才包含已激活学习策略的直接证据。未接管任务的抗扰表现不能单独归功于Actor。

当前保留的失败类型、持续低速、重复侧向摇摆和恢复失败已逐条记录。下一轮修正应选择对应任务ID，固定输入检查原因，再重复同一配对条件。全局图未找到路线、局部控制无法脱困和整队不能归位，需要分别处理。

质点模型未计算姿态、推进器动态和机体载荷。经验成功率、名义制动核验与有限扰动范围共同限定本版已完成的技术证据；不把局部固定参考稳定性推广为整个切换系统的全局定理。

@source E04 E06 E07 E08 E12''')
    p('24 版本归档及可复现交付','''代码版本为2.3.4-unified-actor-20261029，所有正式任务运行源码指纹为debaaedd476be43ecd81eda0675389b3eef73203d9b212f5f471904961782e93。模型文件和参数字节指纹在每条任务记录中保存。

GitHub主分支已提交统一入口、固定模型、原训练绑定源码和完整地图任务包。复现实验从提交的catalogue.mat读取输入，不依赖原开发机旧results目录。原始逐步MAT留在F盘，并通过任务表和文件指纹索引定位。

正式报告、可编辑图源、独立修订记录和任务书反馈分别保存。历史残差ADP报告保留副本，并明确由本轮统一Actor报告替换。教师反馈栏保持空白，未代填验收结论。

@evidence''')
    starts={1,2,8,12,13,15,18,21,24}
    return ''.join(('\n\n@page\n\n' if i in starts else '\n\n' if i else '')+v for i,v in enumerate(pages))

def main():
    global FACTS,ROWS,AUDITS
    complete=json.loads((RUN/'completion.json').read_text(encoding='utf-8'))
    assert complete['complete'] and complete['trials']==3840 and complete['allAuditsPassed']
    assert (RUN/'serial_timing/serial_timing.csv').exists(),'Fresh serial timing is required'
    FACTS=json.loads((RUN/'report_facts.json').read_text(encoding='utf-8'))
    ROWS=[json.loads(p.read_text(encoding='utf-8')) for p in sorted((RUN/'records').glob('*.json'))]
    AUDITS={p.stem:json.loads(p.read_text(encoding='utf-8')) for p in (RUN/'independent_audits').glob('*.json')}
    assert len(ROWS)==3840 and len(AUDITS)==3840
    (ROOT/'_work').mkdir(exist_ok=True)
    make_figures()
    student=student_text().replace('二维参考也在同一机器和单线程设置下另测',timing_table(False)+'\n\n二维参考也在同一机器和单线程设置下另测')
    teacher=teacher_text()
    if (ROOT/'figures/安全层同图轨迹.png').exists():
        student=student.replace('如果安全层提高成功次数，同时增加时间或路径，','如果安全层提高成功次数，同时增加时间或路径，')
        student=student.replace('若安全层提高成功次数，同时增加时间或路径，','@figure 安全层同图轨迹 | 图6 同一地图开启和关闭安全模块的真实轨迹及净空\n\n若安全层提高成功次数，同时增加时间或路径，')
        teacher=teacher.replace('安全距离覆盖全部执行样本。对于发生硬安全距突破的任务，','@figure 安全层同图轨迹 | 图6 同一地图和Actor下的安全模块配对轨迹\n\n安全距离覆盖全部执行样本。对于发生硬安全距突破的任务，')
    if complete.get('executionErrorAttempts',0):
        note=f"本轮有3840条正式任务，另完整保留 {complete['executionErrorAttempts']} 次并发规划内存不足的异常记录。这些任务随后使用相同冻结源码、地图和参数单独串行重跑。正文结果表对应最终任务结局，异常及重试情况另见E14。"
        student=student.replace('本次共重跑3840次已声明任务',note+'\n\n本次共重跑3840次已声明任务')
        teacher=teacher.replace('本轮将所有已知地图默认入口',note+'\n\n本轮将所有已知地图默认入口')
    metadata=json.loads((RUN/'serial_timing/model_metadata.json').read_text(encoding='utf-8'))
    teacher=teacher.replace('原配置中的训练上限和实际导出训练轮数在模型来源记录中分别核对',f"原配置的训练上限为2000轮，当前模型元数据记录实际完成 {metadata['completedTrainingEpisodes']} 轮训练")
    hardware=json.loads((RUN/'hardware.json').read_text(encoding='utf-8'))
    teacher=teacher.replace('正式并发批次全部停止后',f"计时硬件为 {hardware.get('processorName',hardware['processor'])}，系统向进程提供 {hardware['logicalCPUsVisible']} 个逻辑CPU，MATLAB R2024a使用单计算线程。正式并发批次全部停止后")
    student=student.replace('公开的结果表与报告','GitHub上的结果表与报告')
    dense=np.concatenate([np.loadtxt(RUN/'serial_timing'/f'normal_61036_r{k}_cycle_ms.csv',delimiter=',',ndmin=1) for k in [1,2,3]])
    timing_verdict=f"密集37障碍场景的合并P99为 {np.percentile(dense,99):.2f} 毫秒，最大 {dense.max():.2f} 毫秒，{np.sum(dense>50)}/{len(dense)} 个周期超过50毫秒。该场景Actor调用数为0，当前五机完整控制链路的实时计算能力不足。"
    student=student.replace('P99低于50毫秒，表示约99%的观测周期较短，',timing_verdict+'\n\nP99低于50毫秒，表示约99%的观测周期较短，')
    teacher=teacher.replace('周期计时覆盖反馈、监督器、基础控制、安全选令、真实推进、评价和循环内日志赋值；',timing_verdict+'\n\n周期计时覆盖反馈、监督器、基础控制、安全选令、真实推进、评价和循环内日志赋值；')
    recovery_text='\n\n'+recovery_table()+'\n\n这组快速恢复表现偏弱：0.30米/秒平方时19/48次满足5秒恢复要求，1.00米/秒平方时只有3/48次。受扰任务最终都完成，恢复到配对无脉冲轨迹附近的速度仍明显不足。'
    student=student.replace('原始恢复时间与对照任务ID保存在恢复表中。',recovery_text+'\n\n原始恢复时间与对照任务ID保存在恢复表中。')
    teacher=teacher.replace('完成任务和快速恢复分别统计，避免把最终成功替代瞬态性能。','完成任务和快速恢复分别统计。'+recovery_text)
    noise_observation='纯位置噪声的完整A1S1组有14/108次满足重复侧向摇摆判据，联合干扰组有3/144次。当前表现可以完成任务，但仍有明确的轨迹波动。1824次抗扰任务的Actor实际调用数均为0，这些结果反映的是规划、APF、编队及安全闭环；本组没有检验Actor激活后的抗噪表现。'
    student=student.replace('较大的位置噪声会使参考点和控制命令更频繁变化。',noise_observation+'\n\n较大的位置噪声会使参考点和控制命令更频繁变化。')
    teacher=teacher.replace('每轴标准差不是误差硬上界，',noise_observation+'\n\n每轴标准差不是误差硬上界，')
    student=student.replace('原始MAT保存位置、速度、命令、真实总加速度、最近障碍距离、退化标记、安全触发和Actor模式。','完整组50/50完成，8次持续低速后来都恢复了运动；仍有3次满足侧向反复摇摆判据。安全层关闭的Actor组只有38/50完成。\n\n原始MAT保存位置、速度、命令、真实总加速度、最近障碍距离、退化标记、安全触发和Actor模式。')
    student=student.replace('完整组50/50完成，8次持续低速后来都恢复了运动；仍有3次满足侧向反复摇摆判据。安全层关闭的Actor组只有38/50完成。','完整组50/50完成，前述持续低速段后来都恢复了运动。关闭安全层后，同一Actor组只有38/50完成。')
    student=student.replace('图2 同一正式噪声','图3 同一正式噪声').replace('图3 同一安全模块','图4 同一安全模块').replace('图4 同一正式窗口','图5 同一正式窗口').replace('图5 全局规划','图6 全局规划').replace('图6 同一地图开启','图2 同一地图开启')
    teacher=teacher.replace('图2 位置噪声','图3 位置噪声').replace('图3 同一0.10','图4 同一0.10').replace('图4 正式窗口','图5 正式窗口').replace('图5 同图六','图6 同图六').replace('图6 同一地图和Actor','图2 同一地图和Actor')
    (ROOT/'学生版_正文.md').write_text(student,encoding='utf-8');(ROOT/'教师版_正文.md').write_text(teacher,encoding='utf-8')
    layout.build_doc(ROOT/'学生版_正文.md','无人机编队项目整体报告_学生版')
    layout.build_doc(ROOT/'教师版_正文.md','无人机编队项目技术报告_教师版',True)
    evidence=[dict(id=k,label=v[0],path=str(v[1])) for k,v in EVIDENCE.items()]
    (ROOT/'evidence_index.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')
    print('UNIFIED_ACTOR_REPORTS_BUILT',flush=True)
if __name__=='__main__':main()

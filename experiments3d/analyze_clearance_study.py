"""Evidence-backed Chinese report and paired summaries; standard library only."""
from __future__ import annotations
import argparse, csv, html, importlib.util, json, math, statistics
from pathlib import Path

def read(path):
    with Path(path).open(encoding='utf-8-sig',newline='') as f: return list(csv.DictReader(f))
def yes(r): return r['success'] in ('1','true','True')
def numbers(rows,key):
    result=[]
    for row in rows:
        try: v=float(row[key])
        except (ValueError,KeyError): continue
        if math.isfinite(v): result.append(v)
    return result
def describe(rows,key):
    values=numbers(rows,key)
    return dict(n=len(values),mean=statistics.mean(values),min=min(values),max=max(values),median=statistics.median(values)) if values else None
def paired(a,b):
    left={int(r['seed']):r for r in a}; right={int(r['seed']):r for r in b}
    assert set(left)==set(right), 'Paired maps differ'
    rescue=[s for s in left if not yes(left[s]) and yes(right[s])]
    loss=[s for s in left if yes(left[s]) and not yes(right[s])]
    common=[s for s in left if yes(left[s]) and yes(right[s])]
    n=len(left); delta=(len(rescue)-len(loss))/n
    se=math.sqrt(max(0,(len(rescue)+len(loss))/n-delta**2)/n)
    effects={}
    for key in ['time','path','verticalTravel','formationRms','minObstacleClearance','minPairDistance','commandEffort']:
        diffs=[]
        for s in common:
            try: x=float(right[s][key])-float(left[s][key])
            except (ValueError,KeyError): continue
            if math.isfinite(x): diffs.append(x)
        effects[key]=dict(n=len(diffs),mean=statistics.mean(diffs),median=statistics.median(diffs),min=min(diffs),max=max(diffs)) if diffs else None
    return dict(n=n,rescued=len(rescue),lost=len(loss),commonSuccess=len(common),delta=delta,
                delta95Normal=[max(-1,delta-1.95996398454*se),min(1,delta+1.95996398454*se)],
                rescueSeeds=rescue,lossSeeds=loss,pairedEffects=effects)
def write_json(path,data): Path(path).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+
                     ['| '+' | '.join(map(str,r))+' |' for r in rows])
def fmt(v,digits=2): return '—' if v is None else f'{v:.{digits}f}'
def window_layer_probability(z):
    # The frozen generator samples center U[11,19], with 10 m openings.
    radius=5-z; intervals=sorted((max(11,h-radius),min(19,h+radius)) for h in [5,10,15,20,25])
    length=0; end=11
    for lo,hi in intervals:
        if hi>max(lo,end): length+=hi-max(lo,end); end=max(end,hi)
    return length/8

def build(study):
    spec=importlib.util.spec_from_file_location('frozen_clearance_controller',study/'source_snapshot/experiments3d/clearance_study.py')
    controller=importlib.util.module_from_spec(spec); spec.loader.exec_module(controller)
    protocol=json.loads((study/'protocol.json').read_text(encoding='utf-8-sig'))
    selection=json.loads((study/'selection.json').read_text(encoding='utf-8'))
    final=selection['config']; stages={}; summaries={}
    for stage in ['screen','refine','nominal','robust','holdout','boundary','geometry','ablation','timing']:
        rows,summary=controller.collect(study,stage); stages[stage]=rows; summaries[stage]=summary
        assert not any(r['reason']=='execution_error' for r in rows), f'{stage}: numerical execution errors remain'
    cid=final['configId']; baseline=read(study/'baseline_trials.csv')
    nominal=[r for r in stages['nominal'] if r['configId']==cid]
    hold=[r for r in stages['holdout'] if r['configId']==cid]
    control=[r for r in stages['holdout'] if r['configId']=='baseline']
    nominal_pairs=paired(baseline,nominal); hold_pairs=paired(control,hold)
    robust=[]; boundary=[]; geometry=[]; ablation=[]; timing=[]
    for config_id in ['baseline',cid]:
        for noise,dist in sorted({(float(r['noiseStd']),float(r['disturbanceMax'])) for r in stages['robust']}):
            sub=[r for r in stages['robust'] if r['configId']==config_id and float(r['noiseStd'])==noise and float(r['disturbanceMax'])==dist]
            robust.append(dict(configId=config_id,noiseStd=noise,disturbanceMax=dist,n=len(sub),success=sum(yes(r) for r in sub),outcomes=dict(controller.Counter(r['reason'] for r in sub))))
        for axis in ['noiseStd','disturbanceMax']:
            other='disturbanceMax' if axis=='noiseStd' else 'noiseStd'
            sub=[r for r in stages['boundary'] if r['configId']==config_id and float(r[other])==0]
            for strength in sorted({float(r[axis]) for r in sub}):
                part=[r for r in sub if float(r[axis])==strength]
                boundary.append(dict(configId=config_id,axis=axis,strength=strength,n=len(part),success=sum(yes(r) for r in part),outcomes=dict(controller.Counter(r['reason'] for r in part))))
    for config_id,policy in sorted({(r['configId'],r['layerPolicy']) for r in stages['geometry']}):
        sub=[r for r in stages['geometry'] if r['configId']==config_id and r['layerPolicy']==policy]
        geometry.append(dict(configId=config_id,policy=policy,n=len(sub),success=sum(yes(r) for r in sub),
                             flown=sum(int(r['cycles'])>0 for r in sub),outcomes=dict(controller.Counter(r['reason'] for r in sub))))
    for config_id in dict.fromkeys(r['configId'] for r in stages['ablation']):
        sub=[r for r in stages['ablation'] if r['configId']==config_id]
        ablation.append(dict(configId=config_id,n=len(sub),success=sum(yes(r) for r in sub),outcomes=dict(controller.Counter(r['reason'] for r in sub)),
                             parameters={key:float(sub[0][key]) for key in ['enabled','pathXY','refXY','planZ','refZ']}))
    for config_id in ['baseline',cid]:
        sub=[r for r in stages['timing'] if r['configId']==config_id]
        cycles=sum(int(r['cycles']) for r in sub); over=sum(int(r['cyclesOver50ms']) for r in sub)
        timing.append(dict(configId=config_id,n=len(sub),flights=sum(int(r['cycles'])>0 for r in sub),cycles=cycles,over50=over,
                           initialPlan=describe(sub,'initialPlanSeconds'),perFlightP99=describe(sub,'cycleP99ms'),cycleMax=describe(sub,'cycleMaxMs')))
    safety={}
    for stage,rows in stages.items():
        flown=[r for r in rows if int(r['cycles'])>0]
        safety[stage]=dict(n=len(rows),flown=len(flown),outcomes=dict(controller.Counter(r['reason'] for r in rows)),
                           minObstacle=describe(flown,'minObstacleClearance'),minPair=describe(flown,'minPairDistance'))
    coarse=[r for r in stages['screen'] if int(r['enabled'])==1]
    lookup={(int(r['seed']),float(r['pathXY']),float(r['refXY']),float(r['planZ'])):r for r in coarse}
    changes=[]; monotonicity=[]
    for field,position in [('pathXY',1),('refXY',2),('planZ',3)]:
        levels=sorted({float(r[field]) for r in coarse}); counts=controller.Counter()
        for key,left in lookup.items():
            level=levels.index(key[position])
            if level==len(levels)-1: continue
            right_key=list(key); right_key[position]=levels[level+1]; right=lookup[tuple(right_key)]
            counts['comparisons']+=1
            if yes(left)==yes(right): continue
            kind='more_margin_rescued' if yes(right) else 'more_margin_lost'; counts[kind]+=1
            changes.append(dict(axis=field,seed=key[0],leftConfig=left['configId'],rightConfig=right['configId'],
                                lowerMargin=key[position],higherMargin=right_key[position],kind=kind,
                                lowerReason=left['reason'],higherReason=right['reason'],
                                lowerMinObstacle=left['minObstacleClearance'],higherMinObstacle=right['minObstacleClearance']))
        monotonicity.append(dict(axis=field,**dict(counts)))
    if changes: controller.write_csv(study/'nonmonotonic_examples.csv',changes,list(changes[0]))
    result=dict(sourceCommit=protocol['sourceCommit'],studyDir=str(study),final=final,selection=selection,
                scheduled=sum(len(r) for r in stages.values()),stageCounts={s:len(r) for s,r in stages.items()},
                nominalPaired=nominal_pairs,holdoutPaired=hold_pairs,robust=robust,boundary=boundary,
                geometry=geometry,ablation=ablation,timing=timing,safety=safety,coarseMonotonicity=monotonicity)
    write_json(study/'analysis.json',result)
    sections=[]
    def add(title,body): sections.append((title,body))
    nominal_m=controller.metrics(nominal); hold_m=controller.metrics(hold); base_m=controller.metrics(control)
    add('结论与使用条件',f"最终实验候选 `{cid}`：规划水平余量 **{final['pathXY']:g} m**，参考水平余量 **{final['refXY']:g} m**，规划/参考共享竖直余量 **{final['planZ']:g} m**。仍用 5/10/15/20/25 m 固定层，障碍硬安全距 0.6 m、机间硬安全距 1 m。\n\n独立 seed 1001–2000 中，基线 **{base_m['success']}/1000（{base_m['rate']*100:.1f}%）**，候选 **{hold_m['success']}/1000（{hold_m['rate']*100:.1f}%）**；同图救回 {hold_pairs['rescued']} 张，丢失 {hold_pairs['lost']} 张。此结论对应当前四族等量合成地图、质点模型及名义无噪声无外扰条件。边界组中的失败同样保留，不能把名义成功率作为真实飞行保证。")
    add('实验范围与可复现性',f"新执行 **{result['scheduled']} 次任务**；原 2.1.0 的 1000 次记录另作冻结基线。先全域扫描 210 组参数（每组相同 8 图），再从三个分离区域细化，保留两个分离候选进行 1000 图确认和稳健性筛选。最终候选在独立验证运行前冻结。\n\n"+table(['阶段','任务数'],[[s,len(r)] for s,r in stages.items()])+f"\n\n每个任务保留原始 MAT、参数、逐周期状态/参考/噪声/外扰、地图几何哈希、成功或失败原因。所有运行使用源码快照提交 `{protocol['sourceCommit']}`。实际任务协议以各阶段 CSV/JSON 为准，数值源文件哈希以 `source_sha256.csv` 为准。")
    add('三个余量怎样影响系统','规划水平余量决定长机通路预留；参考水平余量决定跟机参考点离障碍的水平距离；共享竖直余量决定规划和参考的竖直障碍膨胀。障碍按水平/竖直半轴构成的椭球膨胀，实际碰撞仍按物理距离判定。降低竖直余量能扩大窗口可用高度区间，降低水平余量能增加水平通路；两者也会减小跟踪误差与扰动的容错空间。\n\n`desiredClear` 同时被旧地图生成器读取，因此本实验直接冻结旧 MAT 的地图，将地图生成参数与待调控制参数分离。同 seed、同地图、同随机序列才形成有效对照。')
    centers=json.loads((study/'refinement_decision.json').read_text(encoding='utf-8'))['screenCenters']
    screened=controller.configs_from_rows(stages['screen']); coarse_table=[]
    for center in centers:
        q=screened[center]; m=summaries['screen'][center]
        coarse_table.append([center,q['pathXY'],q['refXY'],q['planZ'],f"{m['success']}/{m['n']}"])
    confirm_table=[]
    for candidate in json.loads((study/'nominal_decision.json').read_text(encoding='utf-8'))['configs']:
        nc=candidate['configId']; nm=summaries['nominal'][nc]; rm=summaries['robust'][nc]
        unsafe=sum(rm['outcomes'].get(k,0) for k in ['collision','boundary_violation','invalid_state','execution_error'])
        confirm_table.append([nc,candidate['pathXY'],candidate['refXY'],candidate['planZ'],f"{nm['success']}/{nm['n']}",f"{rm['success']}/{rm['n']}",unsafe])
    add('参数选择过程',table(['粗筛中心','规划水平/m','参考水平/m','竖直/m','成功/8'],coarse_table)+'\n\n'+table(['确认候选','规划水平/m','参考水平/m','竖直/m','旧图成功/1000','稳健性组合成功/280','稳健性安全失败数'],confirm_table)+'\n\n三个粗筛区域按参数归一化距离保持分离，各轴细化；两个确认候选继续保持分离。最终排序以冻结的选择规则为准：先少碰撞/无效状态/执行错误，再稳健性、名义和最差场景成功率，最后较大竖直余量。完整分数和决策收据在 `selection.json`；若存在边界越界，也按安全失败披露。稳健性 280 次包含重复地图，只用于候选排序。旧图 1000 张中包含调参地图，因此独立结论以另外 1000 张为准。')
    add('并列候选的解释','本次两个候选的旧图确认和稳健性筛选均全部成功，最终分数完全平局。冻结执行器按配置编号固定排序取 f005 作为独立验证代表，同时保留 f012 对照。两者形成可用参数点的证据，后续可结合机型误差预算和编队形变指标选择；本次没有统计证据证明其中一个在所有场景都更优。')
    add('为什么不能只沿一个方向调参',table(['增加的余量轴','相邻格点同图比较次数','增加后救回','增加后丢失'],[[x['axis'],x['comparisons'],x.get('more_margin_rescued',0),x.get('more_margin_lost',0)] for x in monotonicity])+ '\n\n每次比较仅改变一个余量，其余参数和地图相同；比较的是完整飞行结局。同一张地图参与多次比较，这些计数用于显示非单调性，不作为独立成功率。改变规划余量可能改变离散路线，改变参考余量也改变跟机参考和安全修正，因此不能假设余量越小或越大，任务表现一定越好。逐案例对照见 `nonmonotonic_examples.csv`。')
    add('成功率与独立验证',table(['数据','方案','成功/任务','成功率','Wilson 95% 区间'],[
        ['旧图确认',cid,f"{nominal_m['success']}/1000",f"{100*nominal_m['rate']:.1f}%",f"{100*nominal_m['wilson95'][0]:.2f}%–{100*nominal_m['wilson95'][1]:.2f}%"],
        ['独立验证','baseline',f"{base_m['success']}/1000",f"{100*base_m['rate']:.1f}%",f"{100*base_m['wilson95'][0]:.2f}%–{100*base_m['wilson95'][1]:.2f}%"],
        ['独立验证',cid,f"{hold_m['success']}/1000",f"{100*hold_m['rate']:.1f}%",f"{100*hold_m['wilson95'][0]:.2f}%–{100*hold_m['wilson95'][1]:.2f}%"]])+ '\n\n'+table(['场景','基线','候选'],[[family,f"{base_m['families'][family]['success']}/250",f"{hold_m['families'][family]['success']}/250"] for family in base_m['families']])+f"\n\n同图成功率差：{100*hold_pairs['delta']:.1f} 个百分点；配对正态近似 95% 区间 {100*hold_pairs['delta95Normal'][0]:.2f}–{100*hold_pairs['delta95Normal'][1]:.2f} 个百分点。Wilson 区间描述单方案比例，配对差另算，二者不可混用。验证集在生成时已冻结，但生成地图没有读取规划/飞行结局。")
    add('同图性能和安全距离',f"以下差值为候选减基线，仅使用独立验证两方案共同成功的 **{hold_pairs['commonSuccess']} 张**地图。新增成功另列，避免不同成功样本混在一起导致假改善。\n\n"+table(['指标','配对数量','平均差值','中位差值'],[[k,x['n'],fmt(x['mean'],4),fmt(x['median'],4)] for k,x in hold_pairs['pairedEffects'].items() if x])+ '\n\n'+table(['独立验证飞行指标','基线最小值','候选最小值'],[[key,fmt(min(numbers(control,key)),4),fmt(min(numbers(hold,key)),4)] for key in ['minObstacleClearance','minPairDistance']])+ '\n\n升降、路长和平方命令加速度积分为代理指标，尚无机型质量、效率和电池功率模型，不能换算为真实节能百分比。`formationRms` 针对变形后的参考编队，不等同于相对原始 V 形的形状误差；降低参考余量也改变参考本身，需结合真实跟机高度偏差、物理间距和参考阻塞比例解读。')
    add('稳健性筛选',table(['方案','位置噪声标准差/m','外扰上界/m/s²','成功/40','结果计数'],[[x['configId'],x['noiseStd'],x['disturbanceMax'],f"{x['success']}/{x['n']}",str(x['outcomes'])] for x in robust])+ '\n\n外扰保持 20 周期，即每 1 s 换一次随机向量，各机方向独立、幅度在 0–上界之间均匀抽取。位置噪声为每周期、每坐标独立的高斯噪声；速度反馈无噪声。不同方案共用噪声/外扰序列。40 张地图跨多个条件重复，不将所有条件合并视作更多独立地图；筛选数据也不算独立验证。高斯噪声没有有限绝对上界，此结果对应所用随机扰动模型。')
    add('系统边界在哪里',table(['方案','边界轴','强度','成功/任务','失败与成功计数'],[[x['configId'],x['axis'],x['strength'],f"{x['success']}/{x['n']}",str(x['outcomes'])] for x in boundary])+ '\n\n每个强度为 8 张地图、2 个独立随机重复，只有 8 张独立地图。只能报告所测离散强度下的观察边界，首次失败不代表更大强度必然全失败，也不能外推唯一安全阈值。安全层根据有噪声的测量状态预测，最终碰撞判定读取真实运动航段；若出现碰撞，记录它并揭示预测安全与真实安全的差距。\n\n终端成功条件含持续 1 s 的队形误差门限，而该误差使用受位置噪声影响的参考点。高噪声下超时可能包含终端判据的噪声敏感性；需结合真实净空、终端位置/速度和参考误差字段解释，不把全部超时当作碰撞或失控。')
    add('固定层与窄窗口边界',table(['方案','高度策略','已起飞/45','完整成功/45','结果计数'],[[x['configId'],x['policy'],f"{x['flown']}/{x['n']}",f"{x['success']}/{x['n']}",str(x['outcomes'])] for x in geometry])+ '\n\n控制窗口竖向开口高度为 1/1.25/1.5/2/3/4/6/8/10 m，中线为 10/11.25/12.5/13.75/15 m。实体横贯工作区，无法绕边或爬过。对于竖直规划余量 z，窗口几何允许中心高度区间为 `[窗口下缘+z, 窗口上缘−z]`；当开口高度小于 2z 时没有满足该余量的通路，等于 2z 时可用带退化为单一高度，缺少竖直误差容错，参考投影的保守数值量也需考虑。即便区间非空，固定五层仍可能全错过；加入窗口中线只作独立诊断，不能写入默认五层成功率。\n\n这里记录到的已起飞次数和完整成功次数不同：小余量与补层可以改善几何通路覆盖，后续的参考生成、跟踪和终端归队仍可能失败。三项都取 0.61 m 是本次探索下限，尚未通过真实机型标定；45 张受控窗口也不属于随机地图总体成功率。')
    aperture_heights=[1,1.25,1.5,2,3,4,6,8,10]
    stagnation=sorted([r for r in stages['geometry'] if r['configId']==cid and r['layerPolicy']=='fixed5' and
                      not yes(r) and int(r['cycles'])>0 and aperture_heights[(int(r['seed'])-900001)//5]>=6],key=lambda r:int(r['seed']))
    if stagnation:
        case=stagnation[0]; position=int(case['seed'])-900001
        result['apertureStagnationCase']=dict(seed=int(case['seed']),jobId=case['jobId'],reason=case['reason'],
            verticalOpeningHeight=aperture_heights[position//5],openingCenter=[10,11.25,12.5,13.75,15][position%5],
            finalLeaderError=float(case['finalLeaderError']),finalMaxSpeed=float(case['finalMaxSpeed']))
        add('层图可行后的局部停滞反例',f"按明确规则选取第一个固定五层候选的已起飞失败案例，要求竖向开口高度至少 6 m：seed **{case['seed']}**，开口 **{aperture_heights[position//5]:g} m**。任务通过初始规划并起飞，最终距目标 **{float(case['finalLeaderError']):.3f} m**，全队最大实际速度仅 **{float(case['finalMaxSpeed']):.3f} m/s**，结局为 `{case['reason']}`。这说明控制链未能继续推进航路，需要结合 APF、路径跟踪、参考生成和重规划分析局部停滞。该地图另附完整轨迹和诊断图；高度选择研究应同时验证控制可达性。")
    theory=[dict(z=z,single=window_layer_probability(z),two=window_layer_probability(z)**2) for z in sorted({.61,.8,1.1,1.5,2,2.25,3.2,4.2,final['planZ']})]
    result['windowSamplingTheory']=theory; write_json(study/'analysis.json',result)
    add('为什么当前随机地图成功率会提高', '冻结生成器 `scenario3d.m` 的结构窗口的竖向开口高度均为 10 m，中线均匀抽取于 11–19 m。可用区间高度为 `10−2z`，固定层间隔 5 m；当 z≤2.5 m 时，每个结构窗口都能采到至少一个可用层。这个性质解释了减少竖直膨胀后的大部分可达性收益，仍需单独验证其他障碍、窗口间连接和完整编队飞行。\n\n'+table(['竖直规划余量/m','单窗口含固定层的理论概率','双窗口均含固定层的理论概率'],[[x['z'],f"{100*x['single']:.1f}%",f"{100*x['two']:.1f}%"] for x in theory])+ '\n\n以上按生成器的均匀中线分布和独立双窗口抽样计算，只描述窗口含层事件，不是算法成功率，也不包含其余混合障碍和控制跟踪。原版 z=4.2 m 时理论比例为 20% 和 4%，与旧记录的 22% 和 3.6% 相符。窗口开口高度改变后该性质会改变，因此 1000 张地图上的成功率必须连同生成器条件一起引用。')
    add('消融与可解释性',table(['方案','新模式','规划水平/m','参考水平/m','规划竖直/m','参考竖直/m','成功/40','结果计数'],[[x['configId'],int(x['parameters']['enabled']),x['parameters']['pathXY'],x['parameters']['refXY'],x['parameters']['planZ'],x['parameters']['refZ'],f"{x['success']}/{x['n']}",str(x['outcomes'])] for x in ablation])+ '\n\n所有方案采用相同地图。方案名称描述相对基线的余量数值变化；启用新模式还会切换分方向规划和公共高度截面参考生成。终端参考高度混合时，即便余量数值相同，公共截面与旧版逐参考高度投影也可能不同，因此此消融反映模块配置组合，未单独隔离终端投影实现的因果贡献。规划通过率与完整任务成功率分别记录；窄窗口中还需要跟机参考、跟踪控制和安全修正共同完成整段飞行。该小样本用于解释模块作用，独立成功率以 1000 图验证为准。')
    add('轻量性与计时',table(['方案','执行地图','实际飞行','初始规划中位/s','各飞行P99中位/ms','最大周期/ms','>50ms周期/总周期'],[[x['configId'],x['n'],x['flights'],fmt(x['initialPlan']['median']),fmt(x['perFlightP99']['median']) if x['perFlightP99'] else '—',fmt(x['cycleMax']['max']) if x['cycleMax'] else '—',f"{x['over50']}/{x['cycles']}"] for x in timing])+ '\n\n四个 MATLAB R2024a 单计算线程进程运行批量阶段；计时阶段单独执行，不复用初始规划图缓存。50 ms 对应 20 Hz 仿真控制周期。初始规划在飞行循环之前，在线周期中可能含终端规划/重规划，因此分别报告。单机统计不能证明硬实时；批量阶段的周期时间只保留原始记录，不用于实时性能结论。有限层图容量为 81×81×5=32,805 节点，实际开放节点/边逐任务保存。')
    audit=json.loads((study/'raw_mat_audit.json').read_text(encoding='utf-8-sig')) if (study/'raw_mat_audit.json').exists() else {'status':'尚未审计'}
    assert audit.get('passed') and audit.get('executionErrors')==0, 'Raw MAT audit must pass before reporting'
    add('数据审计与版本回滚',table(['核查项目','结果'],[['实际地图几何哈希',f"{audit['mapGeometryCount']} 张通过"],['原始 MAT',f"{audit['checked']}/{audit['records']} 条通过"],['记录不一致',len(audit['problems'])],['执行异常',audit['executionErrors']]])+'\n\n审计重新读取物理顶点距离、机间距离、参数及终端保持，并与冻结仿真保存的连续航段检查记录核对。独立顶点审计的适用范围见 [完整审计记录](raw_mat_audit.json)。源码及原始 MAT 完整性清单见 [源码哈希](source_sha256.csv)、[原始 MAT 哈希](raw_mat_sha256.csv)、[完整性收据](file_integrity.json)。\n\n基线标签 `baseline-3d-v2.1.0-20261006`；执行标签 `experiment-clearance-run-v2.2.0-20261006`；完成交付标签 `clearance-study-v2.2.0-20261006`；分支 `experiment/clearance-boundary-20261006`。通过新建工作树读取旧标签即可保留现有实验；Git bundle 保存源码历史，原始 MAT 通过 F 盘冻结目录及归档保存。本轮未推送远端。')
    audited=read(study/'raw_mat_audit.csv'); blend=[r for r in audited if int(r['blendChecks'])>0]
    timeout_tail=[]
    audited_by_job={r['jobId']:r for r in audited if r['stage']=='boundary'}
    for item in boundary:
        if item['configId']!=cid: continue
        failed=[r for r in stages['boundary'] if r['configId']==cid and r['reason']=='timeout' and
                ((item['axis']=='noiseStd' and float(r['disturbanceMax'])==0 and float(r['noiseStd'])==item['strength']) or
                 (item['axis']=='disturbanceMax' and float(r['noiseStd'])==0 and float(r['disturbanceMax'])==item['strength']))]
        if not failed: continue
        tails=[audited_by_job[r['jobId']] for r in failed]
        timeout_tail.append(dict(axis=item['axis'],strength=item['strength'],n=len(failed),
            finalLeaderErrorMedian=statistics.median(numbers(failed,'finalLeaderError')),
            finalMaxSpeedMedian=statistics.median(numbers(failed,'finalMaxSpeed')),
            **{key:statistics.mean(numbers(tails,key)) for key in ['tailFormViolationSteps','tailGoalViolationSteps','tailSpeedViolationSteps','tailReferenceBlockedSteps']}))
    result['timeoutTailAudit']=timeout_tail
    add('超时最后 1 s 的判据核对',table(['边界轴','强度','超时次数','终端目标距中位/m','终端最大速度中位/m/s','队形超限周期均值','目标距超限周期均值','速度超限周期均值','参考阻塞周期均值'],[[x['axis'],x['strength'],x['n'],fmt(x['finalLeaderErrorMedian'],3),fmt(x['finalMaxSpeedMedian'],3),fmt(x['tailFormViolationSteps'],1),fmt(x['tailGoalViolationSteps'],1),fmt(x['tailSpeedViolationSteps'],1),fmt(x['tailReferenceBlockedSteps'],1)] for x in timeout_tail])+'\n\n对候选边界组中每次超时，独立读取最后 20 周期的真实状态和参考日志，分别计数队形误差、真实目标距离、真实速度和参考阻塞门限违反次数。各判据可能在同周期同时违反，计数可重叠。队形判据包含受噪声影响的参考，目标距离和速度读取真实状态；该表可检查噪声下的终端判据与大外扰下的运动状态各自如何影响任务完成。全周期数据和逐任务计数保留在原始 MAT 与审计 CSV。')
    shape_a={int(r['seed']):r for r in audited if r['stage']=='holdout' and r['configId']=='baseline' and yes(r)}
    shape_b={int(r['seed']):r for r in audited if r['stage']=='holdout' and r['configId']==cid and yes(r)}
    shared=sorted(set(shape_a)&set(shape_b)); shape=[]
    for key in ['finalVPairRms','finalVPairMax','referenceVPairRms']:
        a=[float(shape_a[s][key]) for s in shared]; b=[float(shape_b[s][key]) for s in shared]
        shape.append(dict(metric=key,n=len(shared),baselineMean=statistics.mean(a),candidateMean=statistics.mean(b),
                          pairedMeanDifference=statistics.mean(y-x for x,y in zip(a,b))))
    result['pairedFinalShape']=shape
    add('实际编队形状核对',table(['指标/m','同图数量','基线均值','候选均值','配对平均差值'],[[x['metric'],x['n'],fmt(x['baselineMean'],4),fmt(x['candidateMean'],4),fmt(x['pairedMeanDifference'],4)] for x in shape])+ '\n\n以原始 V 形偏移的 10 对机间距离作为形状基准，比较实际终端位置的距离 RMS/最大偏差，并另列终端参考的距离 RMS 偏差。它对整队平移和旋转不敏感，可揭示参考被投影后引起的形状变化；它不是位置跟踪误差，也不自动改变原任务成功判定。与高度同步误差、实际最小间距一起使用。')
    sync=[]
    for dataset,rows in [('nominal',nominal),('holdout',hold)]:
        raw=[r for r in audited if r['stage']==dataset and r['configId']==cid and yes(r)]
        successful_rows=[r for r in rows if yes(r)]
        sync.append(dict(dataset=dataset,n=len(raw),maxFollowerHeight=max(numbers(successful_rows,'followerHeightMax')),
                         meanFollowerHeightRms=statistics.mean(numbers(successful_rows,'followerHeightRms')),
                         maxTransitHeight=max(numbers(raw,'transitHeightGapMax')),
                         maxTerminalHeight=max(numbers(raw,'terminalHeightGapMax'))))
    result['heightSynchronization']=sync
    add('瞬态高度同步边界',table(['候选数据集','任务数','最大真实跟机高度差/m','每次RMS均值/m','航路阶段最大/m','终端阶段最大/m'],[[x['dataset'],x['n'],fmt(x['maxFollowerHeight'],4),fmt(x['meanFollowerHeightRms'],4),fmt(x['maxTransitHeight'],4),fmt(x['maxTerminalHeight'],4)] for x in sync])+ '\n\n这是同一时刻实际跟机与长机的高度差，反映瞬态编队变形；两机可能位于航路不同位置，它不等于单机相对规划曲线的高度误差。共享竖直余量是几何膨胀参数，不能用它直接宣称跟机高度差被限制在同一数值。航路与终端分界由日志中第一次有效 holdAlpha 反推进入终端的周期。\n\n现有成功条件只要求最终持续保持，瞬态高度差另列为性能边界。严格同高飞行需要进一步研究高度同步、爬降速度预算及终端过渡；高度层选择也需考虑这些控制可达性条件。')
    intrusion=[r for r in blend if float(r['blendReferenceMin'])<1-1e-6]
    result['terminalReferenceAudit']=dict(recordsWithHeightBlend=len(blend),inflatedReferenceIntrusions=len(intrusion),
                                        maxHeightBlend=max(numbers(audited,'blendHeightGap')),
                                        minNormalizedReference=min(numbers(blend,'blendReferenceMin')) if blend else None)
    write_json(study/'analysis.json',result)
    add('终端参考生成的边界',f"终端过渡期间，参考高度可能暂时与长机公共投影高度不同。冻结实现使用长机高度的水平截面投影，因此参考余量不是这段过渡中逐参考点完整三维约束的证明。独立审计实际参考点：{len(blend)} 条任务存在该高度混合，其中 {len(intrusion)} 条出现实际参考点进入设定的椭球膨胀区；最大高度差 {result['terminalReferenceAudit']['maxHeightBlend']:.4f} m。对应原始字段为 `blendHeightGap`、`blendReferenceMin`、`blendChecks`。\n\n参考点进入软余量区不等于实际无人机碰撞，真实运动安全结局仍独立记录。该项属于模型边界，后续严格参考约束版本需要逐参考高度计算截面，再单独冻结和验证；本轮统计对应当前冻结实现。")
    add('运动模型的适用范围','当前是半隐式欧拉的二阶质点模型，命令加速度先按模长限制在 3 m/s²，外扰随后相加；每步实际速度再按模长投影到 3 m/s。实际加速度含外扰时可超过命令上限，速度投影也影响大扰动下的仿真表现。模型尚未包含推力、姿态、执行器带宽、通信延迟和机体体积；所测鲁棒性边界对应这套明确的数值动力学。0.6 m 硬障碍距、1 m 机间距和 0.61 m 探索下限需要结合真实机型和误差预算另行标定。')
    add('局限与下一步','这次三参数搜索包括全域网格和三个分离区域细化，仍不是连续参数空间全局最优证明。固定五层存在开口可通过但层图采样不到的结构性失败；继续减小余量也不会创造不存在的通路。更值得研究的后续策略是以结构开口生成少量事件高度，保留固定层兜底，并在路线评估中同时考察净空、累计升降、跟机可达性及计算成本。需先保持本次固定层对照，再新增算法版本验证，避免混淆参数收益与补层收益。')
    figures=[('01_global_screen','全域参数探索'),('02_holdout_success','独立验证分场景成功率'),('03_holdout_outcomes','独立验证结局'),('04_disturbance_boundary','噪声与持续外扰边界'),('05_aperture_boundary','窗口与高度采样边界'),('06_isolated_timing','隔离计时'),('07_clearance_ablation','参数消融'),('08_height_synchronization','真实高度同步与新增成功队列')]
    add('图表索引','\n\n'.join(f"### {label}\n\n![{label}](figures/{name}.png)\n\n[矢量 PDF](figures/{name}.pdf) · [可编辑 FIG](figures/{name}.fig) · [图表源 CSV](figures/{name}_data.csv)" for name,label in figures))
    example_manifest=study/'examples/example_manifest.csv'
    if example_manifest.exists():
        example_rows=read(example_manifest)
        add('真实轨迹案例',table(['案例','seed','结局','选择规则'],[[x['kind'],x['seed'],x['reason'],x['rule']] for x in example_rows])+'\n\n'+ '\n\n'.join(f"### {x['kind']} · seed {x['seed']}\n\n![真实轨迹](examples/{x['kind']}/trajectory_3d.png)" for x in example_rows)+'\n\n图中的俯视图投影了所有高度的障碍，投影重叠不等于三维碰撞。判断通过窗口或顶棚必须同时看三维轨迹、各机高度和真实物理净空曲线。案例按清单中明确记录的顺序规则取出，用于解释观察到的现象；全样本统计保留所有任务。')
    replay_seed=next((int(x['seed']) for x in read(example_manifest) if x['kind']=='staggered_windows'),1004) if example_manifest.exists() else 1004
    add('在 F 盘复现与生成 MP4',f"在 MATLAB 执行：\n\n```matlab\ncd('F:/UAV project/UAV_APF_ADP_Formation');\naddpath('config','src','src3d','experiments3d');\n[r,folder]=replay_clearance_seed3d({replay_seed},true);\n```\n\n该入口读取冻结地图、候选冻结收据和数值源码快照，结果存入项目 F 盘 `results/clearance_replay3d_...` 新目录；成功或失败都保存 MAT，存在飞行日志时输出图表，第二个参数为 true 时输出 `flight_3d.mp4`。seed 1–2000 可按冻结库复现。\n\n`clearance_candidate_params3d()` 返回选定实验预设。直接调用 `main3d(p)` 会根据改变后的 desiredClear 重新生成随机地图，属于新实验；复现本次成功率需要显式使用冻结地图。默认参数仍提供原版兼容模式，本次候选通过具名预设调用。")
    replay_receipt=study/'replay_validation.json'
    if replay_receipt.exists():
        replay=json.loads(replay_receipt.read_text(encoding='utf-8'))
        assert replay['passed']
        result['replayValidation']=replay
        write_json(study/'analysis.json',result)
        add('冻结复现与动画验收',f"通过用户复现入口再次执行 seed {replay['seed']}，对照原始任务的全部非计时历史字段和终端真实状态，误差均不超过 1e-8；周期墙钟计时不作数值相等要求。MP4 已解码并抽帧检查，分辨率 {replay['videoWidth']}×{replay['videoHeight']}，帧率 {replay['videoFrameRate']:g} fps。视频时间标注读取真实仿真日志，播放速度与仿真时间不同。复现额外执行不计入 8422 次实验分母。\n\n[播放示范 MP4](examples/staggered_windows/flight_3d.mp4)\n\n[复现验收记录](replay_validation.json)")
    graphics_receipt=study/'graphics_source_sha256.json'
    if graphics_receipt.exists():
        result['graphicsSource']=json.loads(graphics_receipt.read_text(encoding='utf-8-sig'))
        write_json(study/'analysis.json',result)
    md='# 分方向余量与系统边界实验报告\n\n'+f"版本 2.2.0 · 数据目录 `{study.name}`\n\n"+'\n\n'.join('## '+title+'\n\n'+body for title,body in sections)+'\n'
    (study/'REPORT.md').write_text(md,encoding='utf-8')
    # Plain local HTML with no connector-dependent visualization requests.
    def body_html(text):
        import re
        blocks=[]; lines=text.splitlines(); index=0
        while index<len(lines):
            line=lines[index]
            if line.startswith('```'):
                code=[]; index+=1
                while index<len(lines) and not lines[index].startswith('```'): code.append(lines[index]); index+=1
                blocks.append('<pre><code>'+html.escape('\n'.join(code))+'</code></pre>'); index+=1; continue
            if line.startswith('| '):
                chunk=[]
                while index<len(lines) and lines[index].startswith('| '): chunk.append(lines[index]); index+=1
                parse=lambda s:[c.strip() for c in s.strip('|').split('|')]
                blocks.append('<div class="table"><table><thead><tr>'+''.join('<th>'+html.escape(c)+'</th>' for c in parse(chunk[0]))+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+html.escape(c)+'</td>' for c in parse(row))+'</tr>' for row in chunk[2:])+'</tbody></table></div>'); continue
            video=re.fullmatch(r'\[([^\]]+)\]\(([^)]+\.mp4)\)',line)
            if video:
                url=html.escape(video[2],quote=True)
                blocks.append(f'<video controls preload="metadata" style="width:100%;border-radius:16px" src="{url}"></video><p><a href="{url}">{html.escape(video[1])}</a></p>')
                index+=1; continue
            escaped=html.escape(line)
            escaped=re.sub(r'`([^`]+)`',r'<code>\1</code>',escaped)
            escaped=re.sub(r'\*\*([^*]+)\*\*',r'<strong>\1</strong>',escaped)
            if line.startswith('!['):
                match=re.match(r'!\[([^\]]+)\]\(([^)]+)\)',line)
                if match: escaped=f'<img src="{html.escape(match[2])}" alt="{html.escape(match[1])}">'
            escaped=re.sub(r'\[([^\]]+)\]\(([^)]+)\)',r'<a href="\2">\1</a>',escaped)
            blocks.append('<h3>'+escaped[4:]+'</h3>' if line.startswith('### ') else '<p>'+escaped+'</p>' if line else '')
            index+=1
        return '\n'.join(blocks)
    css='''body{margin:0;background:#f5f5f7;color:#1d1d1f;font:16px/1.85 -apple-system,BlinkMacSystemFont,"Microsoft YaHei",sans-serif}main{max-width:1100px;margin:auto;padding:70px 26px}header{padding:20px 0 60px}h1{font-size:clamp(36px,5vw,62px);line-height:1.18;letter-spacing:-2px;margin:20px 0}h2{font-size:28px;line-height:1.35;margin:0 0 25px}h3{font-size:21px}section{background:white;border-radius:24px;padding:38px;margin:24px 0;box-shadow:0 6px 30px #00000004}.kicker{color:#0071e3;font-size:14px;letter-spacing:3px}.sub{color:#6e6e73}.stats{display:flex;gap:25px;flex-wrap:wrap}.stat{flex:1;min-width:180px;background:white;border-radius:20px;padding:25px}.stat strong{display:block;font-size:42px;line-height:1.2}.stat span{color:#6e6e73}code{background:#f5f5f7;padding:2px 5px;border-radius:5px;overflow-wrap:anywhere}pre{background:#f5f5f7;padding:22px;border-radius:12px;overflow:auto;line-height:1.6}a{color:#0071e3;text-decoration:none}img{width:100%;display:block}table{border-collapse:collapse;width:100%;font-size:14px}th,td{padding:12px 10px;text-align:left;border-bottom:1px solid #eee;white-space:normal}th{background:#f5f5f7}.table{overflow:auto}footer{padding:30px;color:#6e6e73}@media print{body{background:white}section{box-shadow:none;break-inside:avoid}main{padding:0}img{max-height:700px;object-fit:contain}}'''
    doc=f'<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>分方向余量与系统边界</title><style>{css}</style><main><header><div class="kicker">UAV FORMATION · EXPERIMENT 2.2</div><h1>更小的余量。<br>更清楚的系统边界。</h1><p class="sub">固定五层 · 冻结地图 · 联合参数探索 · 独立验证</p><div class="stats"><div class="stat"><strong>{hold_m["rate"]*100:.1f}%</strong><span>候选独立验证成功率</span></div><div class="stat"><strong>{hold_pairs["delta"]*100:+.1f}</strong><span>同图改善 / 百分点</span></div><div class="stat"><strong>{result["scheduled"]:,}</strong><span>新执行任务 / 全部留档</span></div></div></header>'+''.join('<section><h2>'+html.escape(title)+'</h2>'+body_html(body)+'</section>' for title,body in sections)+f'<footer>数值源版本 {protocol["sourceCommit"][:12]} · <a href="REPORT.md">完整 Markdown</a> · <a href="analysis.json">统计数据 JSON</a></footer></main></html>'
    (study/'REPORT.html').write_text(doc,encoding='utf-8')
    print('REPORT_CREATED '+str(study/'REPORT.html'),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('study',type=Path); args=parser.parse_args(); build(args.study.resolve())

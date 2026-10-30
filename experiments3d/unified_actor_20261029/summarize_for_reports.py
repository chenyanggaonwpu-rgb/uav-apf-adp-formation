from pathlib import Path
import json,csv,collections,math,statistics,sys
P=Path(__file__).resolve().parents[2];O=P/'results/unified_actor_20261029'
def dump(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2),encoding='utf-8')
def readcsv(p):
    with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def avg(rows,key):
    v=[r[key] for r in rows if isinstance(r.get(key),(int,float)) and math.isfinite(r[key])]
    return statistics.mean(v) if v else None
def wilson(s,n):
    if n==0:return [None,None]
    z=1.959963984540054;d=1+z*z/n;c=(s/n+z*z/(2*n))/d;r=z*math.sqrt((s/n*(1-s/n)+z*z/(4*n))/n)/d
    return [c-r,c+r]
def group(rows,keys):
    out=[];mapping=collections.defaultdict(list)
    for r in rows:mapping[tuple(r[k] for k in keys)].append(r)
    for key,g in sorted(mapping.items()):
        success=[r for r in g if r['success']];v=dict(zip(keys,key))
        v.update(trials=len(g),successes=len(success),confidence95=wilson(len(success),len(g)),
            outcomes=dict(collections.Counter(r['reason'] for r in g)),actorTrials=sum(r['actorCalls']>0 for r in g),
            actorCalls=sum(r['actorCalls'] for r in g),activations=sum(r['activations'] for r in g),
            minObstacle=min((r['minObstacleDistance'] for r in g if r['minObstacleDistance'] is not None),default=None),
            minPair=min((r['minPairDistance'] for r in g if r['minPairDistance'] is not None),default=None),
            successTime=avg(success,'flightSeconds'),successPath=avg(success,'pathLength'),successRms=avg(success,'formationRms'),
            safetySteps=sum(r['safetySteps'] for r in g),brakeSteps=sum(r['jointBrakeSteps'] for r in g),
            persistentStalls=sum(r['maxStallSeconds']>=3 for r in g),sway=sum(r.get('lateralSway',False) for r in g),
            obstacleViolations=sum(r['obstacleSafetyViolation'] for r in g),pairViolations=sum(r['pairSafetyViolation'] for r in g),
            surfaceContacts=sum(r['surfaceContact'] for r in g),maxCommand=max((r['maxCommand'] for r in g if r['maxCommand'] is not None),default=None),
            maxActualAcceleration=max((r['maxActualAcceleration'] for r in g if r['maxActualAcceleration'] is not None),default=None))
        out.append(v)
    return out
def pairs(rows,campaign,method0,safe0,method1,safe1,policy=None):
    selected=[r for r in rows if r['campaign']==campaign and (policy is None or r['routePolicy']==policy)]
    def arm(m,s):return {(r['mapID'],r['tag'],r['routePolicy']):r for r in selected if r['method']==m and r['safety']==s}
    a,b=arm(method0,safe0),arm(method1,safe1);assert a.keys()==b.keys()
    common=[(a[k],b[k]) for k in a if a[k]['success'] and b[k]['success']]
    res=dict(campaign=campaign,routePolicy=policy,first=f'{method0}_S{int(safe0)}',second=f'{method1}_S{int(safe1)}',
        trials=len(a),firstSuccesses=sum(r['success'] for r in a.values()),secondSuccesses=sum(r['success'] for r in b.values()),
        rescued=sum(not a[k]['success'] and b[k]['success'] for k in a),regressions=sum(a[k]['success'] and not b[k]['success'] for k in a),
        commonSuccesses=len(common),difference={})
    for metric in ['flightSeconds','pathLength','formationRms','minObstacleDistance','minPairDistance','wallSeconds']:
        d=[y[metric]-x[metric] for x,y in common if x.get(metric) is not None and y.get(metric) is not None]
        d=[x for x in d if math.isfinite(x)]
        res['difference'][metric]=statistics.mean(d) if d else None
    return res
def main():
    rows=[json.loads(p.read_text(encoding='utf-8')) for p in sorted((O/'records').glob('*.json'))]
    assert len(rows)==3840 and len({r['id'] for r in rows})==3840
    audits={p.stem:json.loads(p.read_text(encoding='utf-8')) for p in (O/'independent_audits').glob('*.json')}
    for r in rows:
        r.update({k:v for k,v in audits[r['id']].items() if k not in {'id','passed','steps'}})
        r['maxRequestedCommand']=r['maxCommand'];r['maxCommand']=audits[r['id']].get('maximumAppliedCommand',r['maxCommand'])
    pair=[]
    for campaign in ['repair','dynamic','robust']:
        pair.extend([pairs(rows,campaign,'none',False,'actor',False),pairs(rows,campaign,'none',True,'actor',True),pairs(rows,campaign,'actor',False,'actor',True)])
    pair.append(pairs(rows,'integration','none',True,'actor',True));pair.append(pairs(rows,'single','none',True,'actor',True))
    for policy in ['global','local_apf']:
        pair.append(pairs(rows,'factorial','none',True,'actor',True,policy));pair.append(pairs(rows,'factorial','rule',True,'actor',True,policy))
    recoveries=readcsv(O/'recovery.csv')
    recovery_summary=[]
    byid={r['id']:r for r in rows}
    groups=collections.defaultdict(list)
    for v in recoveries:
        r=byid[v['id']];groups[(r['kind'],r['method'],r['safety'],r['gustAmplitude'])].append(v)
    for k,g in sorted(groups.items()):
        yes=[v for v in g if v['recovered']=='True']
        recovery_summary.append(dict(zip(['kind','method','safety','amplitude'],k))|dict(trials=len(g),recovered=len(yes),
            meanConfirmedSeconds=statistics.mean(float(v['confirmedSeconds']) for v in yes) if yes else None,
            reasons=dict(collections.Counter(v['reason'] for v in g))))
    result=dict(overall=group(rows,['campaign','routePolicy','method','safety']),
        repairPopulation=group([r for r in rows if r['campaign']=='repair'],['family','method','safety']),
        dynamicFamily=group([r for r in rows if r['campaign']=='dynamic'],['family','method','safety']),
        robustCategory=group([r for r in rows if r['campaign']=='robust'],['kind','method','safety']),
        robustNoise=group([r for r in rows if r['campaign']=='robust' and r['kind']=='noise'],['noiseStd','method','safety']),
        robustGust=group([r for r in rows if r['campaign']=='robust' and r['kind']=='gust'],['gustAmplitude','method','safety']),
        robustDirection=group([r for r in rows if r['campaign']=='robust' and r['gustAmplitude']>0],['gustDirection','method','safety']),
        integrationFamily=group([r for r in rows if r['campaign']=='integration'],['family','method','safety']),
        factorialFamily=group([r for r in rows if r['campaign']=='factorial'],['family','routePolicy','method']),
        singleFamily=group([r for r in rows if r['campaign']=='single'],['family','method']),paired=pair,recovery=recovery_summary,
        failures=[r for r in rows if not r['success']],
        active=[r for r in rows if r['method']=='actor' and r['actorCalls']>0],
        sways=[r for r in rows if r.get('lateralSway')],
        scope=dict(trials=3840,catalogueItems=528,independentRandomMapClaim=False,
            modelHashes=sorted({r['modelSha256'] for r in rows}),runtimeHashes=sorted({r['runtimeSourceSha256'] for r in rows}),
            steps=sum(r['steps'] for r in rows),actorCalls=sum(r['actorCalls'] for r in rows),
            allIndependentAuditsPassed=all(v['passed'] for v in audits.values())))
    dump(O/'report_facts.json',result);dump(O/'paired_summary.json',pair)
    paired_records=[]
    for campaign in ['repair','dynamic','robust','integration']:
        lookup={(r['mapID'],r['tag'],r['method'],r['safety']):r for r in rows if r['campaign']==campaign}
        keys=sorted({(r['mapID'],r['tag']) for r in rows if r['campaign']==campaign})
        for mapid,tag in keys:
            a=lookup.get((mapid,tag,'actor',False));b=lookup.get((mapid,tag,'actor',True));n=lookup.get((mapid,tag,'none',True))
            if b is None:continue
            v=dict(campaign=campaign,mapID=mapid,tag=tag,actorSafetyOn=b['reason'],actorSafetyOff=a['reason'] if a else '',baselineSafetyOn=n['reason'] if n else '',
                safetyRescued=bool(a and not a['success'] and b['success']),safetyRegressed=bool(a and a['success'] and not b['success']),
                actorRescued=bool(n and not n['success'] and b['success']),actorRegressed=bool(n and n['success'] and not b['success']),
                actorCalls=b['actorCalls'],safetySteps=b['safetySteps'],brakeSteps=b['jointBrakeSteps'])
            for metric in ['flightSeconds','pathLength','formationRms','minObstacleDistance','minPairDistance']:
                v[metric+'WithSafety']=b.get(metric);v[metric+'WithoutSafety']=a.get(metric) if a else None
                v[metric+'Difference']=b[metric]-a[metric] if a and a['success'] and b['success'] and a.get(metric) is not None and b.get(metric) is not None else None
            paired_records.append(v)
    with (O/'paired_records.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(paired_records[0]));writer.writeheader();writer.writerows(paired_records)
    def table(h,rs):return '\n'.join(['| '+' | '.join(h)+' |','| '+' | '.join(['---']*len(h))+' |']+['| '+' | '.join(map(str,r))+' |' for r in rs])
    text='# 统一 Actor 安全层同图对照分析\n\n所有比较均复用同一地图、初态、参数和模型。S0关闭预测修正及共同制动，S1同时开启两者；真实物理推进和独立碰撞评价保持一致。\n\n'
    for camp in ['repair','dynamic','robust']:
        v=next(p for p in pair if p['campaign']==camp and p['first']=='actor_S0' and p['second']=='actor_S1')
        text+=f'## {camp}\n\n同图任务 {v["trials"]} 次，S0成功 {v["firstSuccesses"]}，S1成功 {v["secondSuccesses"]}；安全层救回 {v["rescued"]} 次、引入回退 {v["regressions"]} 次，共同成功 {v["commonSuccesses"]} 次。\n\n'
        text+='共同成功集合的S1减S0平均差：'+json.dumps(v['difference'],ensure_ascii=False)+'。\n\n'
    changed=[r for r in paired_records if r['safetyRescued'] or r['safetyRegressed']]
    text+='## 所有结局变化的任务\n\n'+table(['实验','地图','条件','S0结局','S1结局','救回','回退'],[[r['campaign'],r['mapID'],r['tag'],r['actorSafetyOff'],r['actorSafetyOn'],r['safetyRescued'],r['safetyRegressed']] for r in changed])
    text+='\n\n逐图代价、净空、机距、Actor调用及安全触发详见 paired_records.csv；原始轨迹与输入保留在同名MAT。\n'
    (O/'安全层同图对照分析.md').write_text(text,encoding='utf-8')
    print(json.dumps(dict(trials=len(rows),failures=len(result['failures']),active=len(result['active']),sway=len(result['sways'])),ensure_ascii=False))
if __name__=='__main__':main()

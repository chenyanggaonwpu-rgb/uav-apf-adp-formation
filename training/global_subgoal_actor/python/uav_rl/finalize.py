from pathlib import Path
import collections,csv,json,shutil,time
import numpy as np
from scipy.stats import binomtest
from .io import ROOT,atomic_json,read_json,sha256,digest,stamp
from .evaluate import evaluate,baselines,score


def paired(rows,baseline):
    by={a:{r['case_id']:r for r in rows if r['arm']==a} for a in ['candidate',baseline]}
    ids=sorted(by['candidate']);assert set(ids)==set(by[baseline])
    won=[];lost=[];groups=collections.defaultdict(list)
    for key in ids:
        a,b=by['candidate'][key],by[baseline][key];x=bool(a['success'] and a['safe']);y=bool(b['success'] and b['safe'])
        if x and not y:won.append(key)
        if y and not x:lost.append(key)
        groups[a['group']].append(int(x)-int(y))
    rng=np.random.default_rng(20261110);samples=np.zeros(10000)
    for values in groups.values():
        v=np.asarray(values);samples+=v[rng.integers(0,len(v),size=(10000,len(v)))].mean(1)*len(v)/len(ids)
    discordant=len(won)+len(lost)
    return {'baseline':baseline,'paired_maps':len(ids),'rescued':won,'regressed':lost,'net_success_gain':len(won)-len(lost),
        'success_rate_difference':(len(won)-len(lost))/len(ids),'stratified_map_bootstrap_95_interval':np.quantile(samples,[.025,.975]).tolist(),
        'mcnemar_exact_p':float(binomtest(len(won),discordant,.5).pvalue) if discordant else 1.}


def independent_test(run,workroot,allow_diagnostic=False,on_progress=None):
    run=Path(run);manifest=read_json(run/'run_manifest.json')
    assert allow_diagnostic or not manifest['diagnostic_only'],'Diagnostic runs are not formal generalization evidence.'
    best=read_json(run/'best_model.json');assert best['level']=='full' and sha256(best['model'])==best['model_sha256']
    binding=read_json(run/'dataset_binding.json');cases=read_json(Path(binding['folder'])/'test.json')
    reservation=read_json(Path(binding['folder'])/'reservation.json');assert digest(cases)==reservation['test_digest']==binding['test_digest']
    path=run/'independent_test.json'
    if path.exists():
        old=read_json(path);assert old['selected_model_sha256']==best['model_sha256'],'Do not select a second checkpoint against an already opened test set.'
        return old
    cfg=read_json(run/'configuration.json')
    if manifest['diagnostic_only']:cases=cases[:cfg['validation'].get('diagnostic_case_limit',len(cases))]
    destination=run/'independent_test';destination.mkdir(exist_ok=True)
    commitment=destination/'commitment.json'
    receipt={'selected_model_sha256':best['model_sha256'],'selected_model':best['model'],'test_digest':binding['test_digest'],'test_used_for_selection':False}
    if commitment.exists():assert read_json(commitment)==receipt
    else:atomic_json(commitment,receipt)
    models=baselines();models['candidate']=Path(best['model']);rows,summary=evaluate(cases,models,destination,workroot,cfg,on_progress)
    comparisons=[paired(rows,a) for a in ['global_rule','apf','rule','v2','v4_anchor']]
    primary=sorted([x for x in comparisons if x['baseline'] in cfg['validation']['primary_baselines']],key=lambda x:x['mcnemar_exact_p']);running=0.
    for i,x in enumerate(primary):running=max(running,min(1.,(len(primary)-i)*x['mcnemar_exact_p']));x['holm_adjusted_p_primary']=running
    candidate=summary['candidate'];family_floor=min(x['safe_success']/x['tasks'] for x in candidate['by_family'].values())
    evidence=all(x['stratified_map_bootstrap_95_interval'][0]>0 and x['holm_adjusted_p_primary']<.05 for x in primary)
    safety=candidate['unsafe']<=min(summary[a]['unsafe'] for a in cfg['validation']['primary_baselines'])
    qualified=evidence and safety and candidate['safe_success']/candidate['tasks']>=.85 and family_floor>=.7
    result={**receipt,'timestamp':stamp(),'summary':summary,'paired_comparisons':comparisons,'rows':rows,
        'diagnostic_only':manifest['diagnostic_only'],'qualified_for_promotion':qualified and not manifest['diagnostic_only'],
        'acceptance':{'paired_advantage_over_global_rule_rule_v2':evidence,'no_more_unsafe_tasks_than_primary_baselines':safety,
            'overall_safe_success_target':.85,'minimum_family_safe_success_target':.7,'observed_minimum_family_rate':family_floor},
        'teacher_queries':0,'bc_updates':0,'interpretation':'Freezing preserves the selected model; passing engineering checks does not itself establish policy superiority.'}
    atomic_json(path,result);return result


def freeze(run,output,allow_diagnostic=False):
    run=Path(run);best=read_json(run/'best_model.json');test=read_json(run/'independent_test.json');manifest=read_json(run/'run_manifest.json')
    assert allow_diagnostic or not manifest['diagnostic_only'],'A diagnostic model cannot be exported as a production release.'
    assert best['model_sha256']==test['selected_model_sha256']==sha256(best['model'])
    output=Path(output).resolve();assert not output.exists(),'Freeze destinations are immutable; choose a fresh name.'
    output.mkdir(parents=True);name='actor_global_planner_frozen.mat';shutil.copy2(best['model'],output/name)
    assert sha256(output/name)==best['model_sha256']
    for file in ['configuration.json','run_manifest.json','best_model.json','independent_test.json','dataset_binding.json']:
        shutil.copy2(run/file,output/file)
    receipt={'frozen_at':stamp(),'actor_sha256':best['model_sha256'],'actor_updates_at_selection':best['actor_updates'],'new_bc_updates':0,
        'initialization':manifest['initialization'],'selected_using':'fixed full validation only','test_used_for_selection':False,
        'qualified_for_promotion':test['qualified_for_promotion'],'diagnostic_only':manifest['diagnostic_only'],
        'production_default_automatically_changed':False,'configuration_sha256':sha256(output/'configuration.json')}
    atomic_json(output/'FROZEN.json',receipt)
    (output/'模型状态.txt').write_text(('仅用于联测，禁止作为训练完成或优势证明。' if manifest['diagnostic_only'] else
        '已冻结验证集选出的研究模型。部署资格以FROZEN.json和独立测试为准；冻结动作本身不代表已通过性能验收。')+'\n',encoding='utf-8')
    return receipt


def report(run):
    run=Path(run);test=read_json(run/'independent_test.json');best=read_json(run/'best_model.json');cfg=read_json(run/'configuration.json')
    labels={'global_rule':'全图规则规划 plan_layers3d + APF','apf':'纯 APF','rule':'局部规则脱困航点 + APF',
        'v2':'V2 局部脱困 + APF','v4_anchor':'旧 V4 局部脱困 + APF','candidate':'全程全局子目标 Actor + APF'}
    lines=['# 全局子目标规划 Actor 独立测试记录','',f"记录时间：{test['timestamp']}。随机初始化；BC=0；专家查询=0。",'',
        '仅联测，不作为泛化证据。' if test['diagnostic_only'] else '固定全验证选模后才打开独立测试；测试不选模。','',
        '| 方法 | 任务 | 安全五机完成 | 安全违规 | 失败计120秒均时 | leader均路程/m | 全队均路程/m | 停滞均时/s | 重访均时/s |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for arm,x in test['summary'].items():
        lines.append(f"| {labels[arm]} | {x['tasks']} | {x['safe_success']} | {x['unsafe']} | {x['failure_counted_120_mean']:.2f} | {x['mean_leader_path_meters']:.2f} | {x['mean_fleet_path_meters']:.2f} | {x['mean_stagnant_seconds']:.2f} | {x['mean_revisit_seconds']:.2f} |")
    lines+=['','| 同图配对对照 | 救回 | 退步 | 净增成功 | 成功率差95%区间 | McNemar p | Holm p（主要对照） |','|---|---:|---:|---:|---|---:|---|']
    for x in test['paired_comparisons']:
        adjusted=x.get('holm_adjusted_p_primary')
        lines.append(f"| {labels[x['baseline']]} | {len(x['rescued'])} | {len(x['regressed'])} | {x['net_success_gain']} | {x['stratified_map_bootstrap_95_interval']} | {x['mcnemar_exact_p']:.5g} | {adjusted if adjusted is not None else '描述性'} |")
    lines+=['','| 场景 | 方法 | 任务 | 安全五机完成 | 安全违规 |','|---|---|---:|---:|---:|']
    for arm,x in test['summary'].items():
        for family,f in x['by_family'].items():lines.append(f"| {family} | {labels[arm]} | {f['tasks']} | {f['safe_success']} | {f['unsafe']} |")
    lines+=['','主要对照为全图规则规划、局部规则脱困航点、V2，三项使用 Holm 校正。',
        '各方法共享同图、同噪声、同风和同扰动种子，统一从 t=0 开始任务，没有旧的陈旧航点故障或75秒晚激活条件。',
        '候选 leader 仅跟踪 Actor 的全局子目标；没有 leader 规则路径。公共终端从机规划、安全层和五机完成条件在所有方法中保留。',
        '全局 Actor 每1.6秒或到达/停滞事件重选子目标，最短0.4秒；终端尾段归入最后一条转移，整任务只有一个 terminal。',
        '标量地图距离势能不提供专家动作或见证路线标签。冻结不自动部署或替换生产 V2。','',
        f"模型 SHA256：{best['model_sha256']}；选定版本 Actor 更新：{best['actor_updates']}。",'',
        f"达到预设部署候选目标：{test['qualified_for_promotion']}。目标为整体安全成功率≥85%、每类≥70%、三个主要对照的配对差值95%区间下界>0且Holm p<0.05、安全违规不增。",'']
    (run/'独立测试报告.md').write_text('\n'.join(lines),encoding='utf-8')
    keys=sorted({k for row in test['rows'] for k in row})
    with (run/'独立测试明细.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(test['rows'])
    return run/'独立测试报告.md'

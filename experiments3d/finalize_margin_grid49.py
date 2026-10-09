"""Package the reviewed figures and offline viewer without changing numeric inputs."""
from __future__ import annotations
import json,shutil,sys
from pathlib import Path
import margin_grid49 as m
from render_margin_surface import render

def copy_baseline_reference(study):
    baseline=Path(json.loads((study/'protocol.json').read_text(encoding='utf-8'))['baseStudy'])
    prior_audit=json.loads((baseline/'raw_mat_audit.json').read_text(encoding='utf-8'))
    assert prior_audit['passed'] and prior_audit['records']==200 and prior_audit['success']==80
    prior_hashes={int(row['seed']):row['rawMatSha256'] for row in m.read_csv(baseline/'raw_mat_audit.csv')}
    directory=study/'baseline_reference';raw=directory/'raw_reference';raw.mkdir(parents=True,exist_ok=True)
    manifest=[]
    for seed in range(2001,2201):
        owner=(seed-2001)//50+1
        original=baseline/'workers'/f'worker_{owner:02d}'/'trials'/f'random_{seed:06d}.mat'
        expected=prior_hashes[seed];assert m.digest(original)==expected
        destination=raw/original.name
        if not destination.is_file() or m.digest(destination)!=expected:shutil.copy2(original,destination)
        assert m.digest(destination)==expected
        manifest.append(dict(seed=seed,sourceFile=original.as_posix(),referenceFile=destination.relative_to(study).as_posix(),
                             sha256=expected,bytes=destination.stat().st_size))
    for name in ['raw_mat_audit.json','raw_mat_audit.csv','source_sha256.csv','protocol.json']:
        shutil.copy2(baseline/name,directory/name)
    shutil.copytree(baseline/'source_snapshot',directory/'source_snapshot',dirs_exist_ok=True)
    for row in m.read_csv(directory/'source_sha256.csv'):
        assert m.digest(directory/'source_snapshot'/row['file'])==row['sha256']
    m.write_csv(study/'baseline_reference_manifest.csv',manifest)
    m.write_json(study/'validation/baseline_reference_packaging.json',dict(passed=True,records=200,
        allCopiesMatchPriorAuditedSha256=True,sourceSnapshotIncluded=True,
        note='Reference records are retained outside the 9800-task denominator. Their r.history can be compared with baseline canonical results after removing cycleMs.'))
    return manifest

def auxiliary_provenance(study):
    protocol=json.loads((study/'aux_execution_protocol.json').read_text(encoding='utf-8'))
    receipt=json.loads((study/'aux_execution_receipt.json').read_text(encoding='utf-8'))
    assert receipt['passed'] and all(code==0 for code in receipt['exitCodes'])
    source_files=list(protocol['sourceFiles'])
    maximum_concurrent=protocol['maxConcurrentProcesses']
    more_receipt=None
    if (study/'more_aux_execution_protocol.json').is_file():
        more_protocol=json.loads((study/'more_aux_execution_protocol.json').read_text(encoding='utf-8'))
        more_receipt=json.loads((study/'more_aux_execution_receipt.json').read_text(encoding='utf-8'))
        assert more_receipt['passed'] and all(code==0 for code in more_receipt['exitCodes'])
        source_files+=more_protocol['sourceFiles']
        maximum_concurrent=max(maximum_concurrent,more_protocol['maxConcurrentProcesses'])
    for item in source_files:
        assert m.digest(study/item['file'])==item['sha256'],item['file']
    audited={row['jobId']:row['rawMatSha256'] for row in m.read_csv(study/'raw_mat_audit.csv')}
    published=[];publication_receipts=[]
    for path in sorted((study/'auxiliary_runs').glob('aux_*/published_*.json')):
        value=json.loads(path.read_text(encoding='utf-8'))
        assert len(value['results'])==49
        assert value['published']==sum(bool(row['published']) for row in value['results'])
        publication_receipts.append(dict(file=path.relative_to(study).as_posix(),sha256=m.digest(path)))
        for row in value['results']:
            if row['published']:
                assert audited[row['jobId']]==row['sha256'],row['jobId']
                published.append(row['jobId'])
    assert len(published)==len(set(published))
    assert len(publication_receipts)>=receipt['publishedMaps']
    if more_receipt is not None:assert len(publication_receipts)==more_receipt['publishedMaps']
    pending=list((study/'auxiliary_runs').glob('aux_*/pending_results/*.mat'))
    value=dict(passed=True,canonicalTasks=9800,maxConcurrentProcesses=maximum_concurrent,
               publishedMaps=len(publication_receipts),canonicalTasksFromAuxiliary=len(published),
               auxiliarySavedTasks=len(pending),auxiliarySavedTasksOutsideCanonical=len(pending)-len(published),
               frozenNumericalSourceUnchanged=True,publishedRawHashesMatchCanonicalAudit=True,
               publicationReceipts=publication_receipts,
               interpretation='Auxiliary saved records outside the canonical population are retained in the ZIP. They are scheduling overlap or partial map work, never additional independent seeds.')
    m.write_json(study/'validation/auxiliary_provenance.json',value)
    return value

def stage(study):
    study=Path(study);m.verify(study)
    audit=json.loads((study/'raw_mat_audit.json').read_text(encoding='utf-8'))
    assert audit['passed'] and audit['records']==9800
    auxiliary=auxiliary_provenance(study)
    copy_baseline_reference(study)
    render(study)
    source=study/'postprocessing_source';source.mkdir(exist_ok=True)
    for name in ['render_margin_surface.py','finalize_margin_grid49.py','plot_margin_grid49_reviewed3d.m','check_margin_viewer.cjs','check_margin_frontend.cjs']:
        shutil.copy2(m.ROOT/'experiments3d'/name,source/name)
    m.write_csv(study/'postprocessing_source_sha256.csv',[
        dict(file=f.relative_to(study).as_posix(),sha256=m.digest(f)) for f in sorted(source.rglob('*')) if f.is_file()])
    logs=study/'validation/runtime_logs';logs.mkdir(exist_ok=True)
    for f in (m.RUNTIME/study.name).rglob('*.log'):
        relative=f.relative_to(m.RUNTIME/study.name);target=logs/relative;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(f,target)
    destination=m.ROOT/'交付文件/余量49组合扫描_20261008'
    report=(study/'REPORT.md').read_text(encoding='utf-8')
    report=report.split('\n## 交互曲面与复核\n',1)[0]
    report+='\n## 交互曲面与复核\n\n'
    links=[('可旋转三维曲面','margin_success_surface.html'),('总体成功率三维图','figures/01_success_surface_all200.png'),
           ('初始安全子集三维图','figures/02_success_surface_safe157.png'),('精确数值热力图','figures/03_success_heatmaps.png'),
           ('飞行失败热力图','figures/04_flight_failure_heatmaps.png'),('49组汇总','grid_summary.csv'),
           ('9800条逐任务记录','all_results.csv'),('完整实验数据ZIP','margin_grid49_full_data.zip'),
           ('Git历史与回滚标签','uav_margin_grid49_history.bundle')]
    report+='\n'.join(f'- [{label}](<{(destination/name).as_posix()}>)' for label,name in links)+'\n'
    report+='\n四张 PNG 已逐图检查，曲面和热力图数值与汇总 CSV 一致。PDF 由 MATLAB 导出，含部分嵌入位图，不称纯矢量图。浏览器自动检查工具在本机启动失败，网页代码与嵌入数据已单独复核，实际浏览器排版和交互尚未通过自动核验；记录见 `validation/viewer_qa/`。\n\n参数默认值保持基线；本轮提供有限网格扫描证据，尚未据此修改推荐预设。曲面页面使用嵌入的审阅 CSV 数据，可离线旋转、缩放和逐点查看成功率、Wilson 区间及失败计数。\n'
    report+='\n## 执行与调度记录\n\n'
    report+=f'最初使用 6 个 MATLAB 单计算线程进程，运行中根据空余内存逐步增加辅助进程，最多同时 {auxiliary["maxConcurrentProcesses"]} 个。辅助进程提前完成后段地图，发布 {auxiliary["publishedMaps"]} 张完整地图的检查点，其中 {auxiliary["canonicalTasksFromAuxiliary"]} 条进入原定 9800 条结果；原进程逐条审核了这些原始 MAT。\n\n'
    report+=f'辅助目录另有 {auxiliary["auxiliarySavedTasksOutsideCanonical"]} 条保存记录未进入统计分母，作为调度重叠或未完整交接地图的证据保留在 ZIP。冻结的数值核心、参数网格及 200 张地图均未修改，49 个组合各有且仅有同一组 200 个 seed。辅助源码、调度差异、发布记录及 SHA-256 随完整数据保留。\n'
    report+='\n完整 ZIP 还保留前一轮的 200 条基线原始记录与源码，位于 `baseline_reference/`，其逐文件 SHA-256 与前一轮审核表一致；这些复核资料不计入 9800 个任务。`baseline_reference_manifest.csv` 给出对应 seed 和包内路径，脱离原实验目录后仍可检查基线轨迹。\n'
    (study/'REPORT.md').write_text(report,encoding='utf-8')
    m.deliver(study)
    docs=m.ROOT/'docs/three_dimensional/margin_grid49_20261008'
    for target in [destination,docs]:
        shutil.copy2(study/'margin_success_surface.html',target/'margin_success_surface.html')
        shutil.copy2(study/'postprocessing_source_sha256.csv',target/'postprocessing_source_sha256.csv')
        shutil.copy2(study/'baseline_reference_manifest.csv',target/'baseline_reference_manifest.csv')
        shutil.copy2(study/'raw_mat_audit.csv',target/'raw_mat_audit.csv')
        shutil.copytree(source,target/'postprocessing_source',dirs_exist_ok=True)
        for name in ['aux_execution_protocol.json','aux_execution_receipt.json']:
            shutil.copy2(study/name,target/name)
        shutil.copytree(study/'aux_runtime_source',target/'aux_runtime_source',dirs_exist_ok=True)
        if (study/'more_aux_execution_protocol.json').is_file():
            for name in ['more_aux_execution_protocol.json','more_aux_execution_receipt.json']:
                shutil.copy2(study/name,target/name)
            shutil.copytree(study/'more_aux_runtime_source',target/'more_aux_runtime_source',dirs_exist_ok=True)
    print('MARGIN_VIEWER_AND_PACKAGING_STAGED',flush=True)

if __name__=='__main__':stage(sys.argv[1])

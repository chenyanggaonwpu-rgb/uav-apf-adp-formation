"""Audit and summarize a completed, frozen fixed-layer seed sweep."""
from __future__ import annotations
import csv
import hashlib
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows, fields):
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def flag(value):
    return str(value).lower() in ("1", "true")


def number(value):
    try:
        v = float(value)
        return v if math.isfinite(v) else None
    except (TypeError, ValueError):
        return None


def wilson(k, n):
    if not n:
        return [None, None]
    z = 1.959963984540054
    p = k / n
    den = 1 + z*z/n
    center = (p + z*z/(2*n))/den
    half = z*math.sqrt(p*(1-p)/n + z*z/(4*n*n))/den
    return [max(0, center-half), min(1, center+half)]


def stats(rows):
    n = len(rows)
    k = sum(flag(r["success"]) for r in rows)
    return {"scheduled": n, "success": k, "failure": n-k,
            "successRate": k/n if n else None, "wilson95": wilson(k, n),
            "reasons": dict(sorted(Counter(r["reason"] for r in rows).items()))}


def pct(value):
    return "不可用" if value is None else f"{100*value:.4f}%"


def finite_mean(rows, key):
    vals = [v for r in rows if (v := number(r.get(key))) is not None]
    return statistics.mean(vals) if vals else None


def main(study):
    root = study.parent.parent
    protocol = json.loads((study/"protocol.json").read_text(encoding="utf-8"))
    planned = list(range(protocol["firstSeed"], protocol["lastSeed"]+1))
    statuses = []
    rows = []
    for index in range(1, protocol["nWorkers"]+1):
        worker = study/"workers"/f"worker_{index:02d}"
        status = json.loads((worker/"status.json").read_text(encoding="utf-8"))
        if status["status"] != "complete":
            raise RuntimeError(f"Worker {index} is not complete")
        statuses.append(status)
        rows += read_csv(worker/"trials.csv")
    rows.sort(key=lambda r: int(r["seed"]))
    if [int(r["seed"]) for r in rows] != planned:
        raise RuntimeError("Missing, duplicate, or unexpected seed")
    mats = list((study/"workers").glob("worker_*/trials/*.mat"))
    if len(mats) != len(planned):
        raise RuntimeError("MAT count does not match scheduled seeds")
    for index, (lo, hi) in enumerate(protocol["ranges"], 1):
        for seed in range(lo, hi+1):
            p = study/"workers"/f"worker_{index:02d}"/"trials"/f"C_random_{seed:06d}.mat"
            if not p.is_file() or not p.stat().st_size:
                raise RuntimeError(f"Missing/empty MAT for seed {seed}")
    fields = list(rows[0])
    write_csv(study/"trials_all_1000.csv", rows, fields)
    failures = [r for r in rows if not flag(r["success"])]
    write_csv(study/"failures.csv", failures, fields)
    trigger_rows = [r for r in rows if int(r["safetyTriggers"]) > 0]
    write_csv(study/"safety_trigger_cases.csv", trigger_rows, fields)

    # Compare the numerical sources to the prior 150-map C baseline.
    baseline_dir = root/"results"/"3d_study_v2.0.0_20261003_221816_234"
    old_manifest = read_csv(baseline_dir/"source_sha256.csv")
    omit = {"run_study3d.m", "src/create_run_output_dir.m", "src3d/plot_result3d.m"}
    compat = []
    for old in old_manifest:
        rel = old["relativePath"].replace("\\", "/")
        if rel in omit:
            continue
        digest = hashlib.sha256((study/"source_snapshot"/rel).read_bytes()).hexdigest()
        compat.append({"relativePath": rel, "priorSha256": old["sha256"],
                       "sweepSha256": digest, "match": digest == old["sha256"]})
    write_csv(study/"baseline_source_compatibility.csv", compat, list(compat[0]))
    baseline = [r for r in read_csv(baseline_dir/"trials.csv")
                if r["group"] == "C" and r["mapKind"] == "random"
                and number(r["noiseStd"]) == 0 and number(r["disturbanceMax"]) == 0]
    baseline.sort(key=lambda r: int(r["seed"]))
    compatible = (all(r["match"] for r in compat)
                  and [int(r["seed"]) for r in baseline] == list(range(1, 151))
                  and protocol["firstSeed"] == 151)
    current = stats(rows)
    prior = stats(baseline)
    pooled = stats(baseline+rows) if compatible else None
    generated = [r for r in rows if flag(r["generationOK"])]
    success_rows = [r for r in rows if flag(r["success"])]
    audit = []
    for item in read_csv(study/"source_sha256.csv"):
        rel = item["relativePath"].replace("\\", "/")
        actual = hashlib.sha256((study/"source_snapshot"/rel).read_bytes()).hexdigest()
        audit.append(actual == item["sha256"])
    if not all(audit):
        raise RuntimeError("A frozen source file changed during the sweep")
    summary = {
        "studyDir": str(study), "seedRange": [planned[0], planned[-1]],
        "group": "C", "noiseStd": 0, "disturbanceMax": 0,
        "current": current, "priorC150": prior, "pooled": pooled,
        "baselineNumericalSourceMatches": sum(r["match"] for r in compat),
        "baselineNumericalSourceCompared": len(compat),
        "frozenSourceAudit": {"matched": sum(audit), "total": len(audit)},
        "generatedMapCount": len(generated),
        "successConditionalOnGeneration": stats(generated),
        "planStatuses": dict(Counter(r["planStatus"] for r in rows)),
        "totalCycles": sum(int(r["cycles"]) for r in rows),
        "safetyTriggerCycles": sum(int(r["safetyTriggers"]) for r in rows),
        "safetyTriggerSeeds": [int(r["seed"]) for r in trigger_rows],
        "errorFileCount": len(list((study/"workers").glob("worker_*/trials/*.error.txt"))),
        "matCount": len(mats), "rawMatMiB": sum(p.stat().st_size for p in mats)/2**20,
        "workerWallSeconds": [s["wallSeconds"] for s in statuses],
        "meanSuccessTime": finite_mean(success_rows, "time"),
        "meanSuccessPath": finite_mean(success_rows, "path"),
        "meanSuccessFormationRms": finite_mean(success_rows, "formationRms"),
        "failureSeeds": [int(r["seed"]) for r in failures],
        "timingNote": protocol["timingRule"],
    }
    analysis_bytes = Path(__file__).read_bytes()
    (study/"analysis_source.py").write_bytes(analysis_bytes)
    summary["analysisSourceSha256"] = hashlib.sha256(analysis_bytes).hexdigest()
    (study/"summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    interval = current["wilson95"]
    lines = [
        "# 固定五高度层随机种子扩展试验", "",
        "日期：2026-10-05。版本：2.0.0。目的：测量当前随机地图规则下的整套三维编队任务成功率。", "",
        f"新增 seed {planned[0]}–{planned[-1]}，预定 **{current['scheduled']} 次**；"
        f"成功 **{current['success']} 次**、失败 **{current['failure']} 次**，成功率 **{pct(current['successRate'])}**。"
        f"Wilson 95% 区间为 **{pct(interval[0])}–{pct(interval[1])}**。", "",
        "## 任务口径", "",
        "保持五层 5/10/15/20/25 m、1 m 水平栅格、4.2 m 长机规划净空，"
        "速度和命令加速度模长上限 3 m/s、3 m/s²、dt=0.05 s、Tmax=120 s。"
        "使用默认 C 组（ADP 与预测安全开启），位置噪声与外部加速度均为零；每个 seed 重置共享学习权重。", "",
        "每张图尝试生成 8 个球体或有限圆柱；每个障碍最多尝试 300 次放置。"
        "生成、初态、规划、碰撞、安全修正未解决、重规划、超时和执行错误均保留在预定 seed 分母里。", "",
        "成功要求长机目标误差不超过 1 m、最大僚机动态参考误差不超过 1 m、"
        "全队最大速度不超过 0.25 m/s，且条件持续 1 s。所有判断由冻结的 run_simulation3d.m 执行。", "",
        "## 结局", "", "| 结局 | 数量 | 比例 |", "| --- | ---: | ---: |",
    ]
    for reason, n in current["reasons"].items():
        lines.append(f"| {reason} | {n} | {pct(n/current['scheduled'])} |")
    lines += ["", f"生成成功地图 {len(generated)} 张；生成成功条件下的飞行成功率为 "
              f"{pct(summary['successConditionalOnGeneration']['successRate'])}。", ""]
    lines += [f"初始规划为图路径 {summary['planStatuses'].get('path',0)} 次、直接路线 "
              f"{summary['planStatuses'].get('direct',0)} 次。成功任务平均完成时间 "
              f"{summary['meanSuccessTime']:.3f} s、长机路径 {summary['meanSuccessPath']:.3f} m、"
              f"动态队形 RMS {summary['meanSuccessFormationRms']:.4f} m。", "",
              f"预测安全层触发 {summary['safetyTriggerCycles']} 个周期，涉及 seed "
              f"{', '.join(str(r['seed']) for r in trigger_rows) if trigger_rows else '无'}。"
              "这些 seed 本轮的结局见 safety_trigger_cases.csv；触发次数本身不能证明关闭安全层会失败。", ""]
    if pooled:
        ci = pooled["wilson95"]
        lines += ["## 与原 150 个随机种子的关系", "",
                  f"影响数值结果的 {len(compat)} 个参数/地图/规划/控制/评价源文件 SHA256 "
                  f"全部匹配。原 C 组 seed 1–150 成功 {prior['success']}/{prior['scheduled']}。"
                  "旧图形标题修改和新动画函数不参与本轮数值仿真。", "",
                  f"合并不重复的 seed 1–{planned[-1]} 后，成功 **{pooled['success']}/{pooled['scheduled']}**，"
                  f"成功率 **{pct(pooled['successRate'])}**，Wilson 95% 区间 "
                  f"**{pct(ci[0])}–{pct(ci[1])}**。只合并 C 组随机地图，不计 A/B、固定地图或噪声试验。", ""]
    lines += ["## 本次能回答什么", "",
              "这里的成功率仅对应当前随机地图分布和默认起终点，不能解释为任意三维场景成功率。"
              "地图生成器把障碍中心 x/y 限制在 15–65 m，半径不超过 4 m，"
              "并要求长机起终点障碍净空至少 5.6 m。空间边缘因此留有稳定绕行通道。", "",
              "按现有 4.2 m 规划净空，z=10 m 层存在以下保守图路径："
              "(10,10,10) → (5,10,10) → (5,75,10) → (75,75,10) → (75,62,10) → (70,62,10)。"
              "x=5、y=75、x=75 三条边缘段与任何障碍表面至少相距 6 m；"
              "起点向左、终点向右的接入段远离所有障碍。该路线节点位于栅格和边界内。"
              "所以生成成功的当前地图在几何上已有同一高度的可行路线，"
              "增加 seed 不足以检验高度层采样的路径完备性。", "",
              "若本次出现失败，要先看日志归因。飞行超时或僚机归队失败，不自动意味着五层漏掉了可行高度。"
              "研究高度选择还需另建边缘封堵、层间安全窗口或受限升降通道场景，"
              "并独立确认连续空间有可行路线。", "",
              "## 失败案例", ""]
    if failures:
        lines += ["| seed | 结局 | 仿真时间/s | 最终目标误差/m | 最终队形误差/m |", "| --- | --- | ---: | ---: | ---: |"]
        for r in failures[:30]:
            lines.append(f"| {r['seed']} | {r['reason']} | {r['time']} | {r['finalLeaderError']} | {r['finalFormationError']} |")
        lines += ["", "所有失败见 failures.csv；完整原始轨迹、地图、参数和错误信息保存在对应 trial MAT。"]
    else:
        lines.append("本次预定种子没有失败；这不代表全环境失败概率为零，也不能证明固定五层完备。")
    for r in trigger_rows:
        case_dir = study/"cases"/f"seed_{r['seed']}"
        if (case_dir/"evidence.json").is_file():
            info = json.loads((case_dir/"evidence.json").read_text(encoding="utf-8"))
            lines += ["", f"## 安全层触发示例：seed {r['seed']}", "",
                      f"该次任务成功，安全层触发 {r['safetyTriggers']} 个周期；规划航点高度为 "
                      f"{info['routeHeights']} m。实飞最小障碍净空 {float(r['minObstacleClearance']):.4f} m，"
                      f"最小机间距离 {float(r['minPairDistance']):.4f} m。图由冻结 MAT 再生，未重新仿真。", "",
                      f"![seed {r['seed']} 轨迹](cases/seed_{r['seed']}/trajectory_3d.png)", "",
                      f"![seed {r['seed']} 诊断](cases/seed_{r['seed']}/diagnostics_3d.png)"]
    lines += ["", "## 数据与计算记录", "",
              f"结果根目录：`{study}`。", "",
              f"核验：{len(rows)} 个唯一 seed、{len(mats)} 个非空 MAT；"
              f"{summary['errorFileCount']} 个错误文件；冻结源文件 {sum(audit)}/{len(audit)} 校验匹配，"
              f"原始 MAT 共 {summary['rawMatMiB']:.2f} MiB。", "",
              "四个独立 MATLAB R2024a 进程使用 singleCompThread 运行各自预定范围。"
              f"最长 worker 工作阶段约 {max(summary['workerWallSeconds'])/60:.2f} 分钟（不含进程启动）。"
              "本次测任务成功率，进程并行期间的逐周期耗时不作实时性能结论。", "",
              "- protocol.mat / protocol.json：运行前固定的 seeds、范围和参数。",
              "- source_snapshot/ 与 source_sha256.csv：实际执行源码及哈希。",
              "- workers/worker_*/trials/*.mat：逐 seed 结果、地图、初始参数、行指标、异常。",
              "- trials_all_1000.csv / failures.csv：完整与失败明细。",
              "- safety_trigger_cases.csv：安全层触发种子。",
              "- summary.json：成功率、Wilson 区间、结局和来源核验。",
              "- baseline_source_compatibility.csv：与旧 C 组 150 种子的数值源码比较。", ""]
    (study/"SEED_SWEEP_REPORT_20261005.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main(Path(sys.argv[1]).resolve())

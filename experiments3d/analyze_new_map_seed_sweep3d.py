"""Read the frozen 2.1.0 fixed-height seed 1--1000 experiment, without reruns."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path


FAMILIES = ("mixed", "underpass", "window", "staggered_windows")
LABELS = {"mixed": "混合三维障碍", "underpass": "顶棚下穿", "window": "单窗口墙", "staggered_windows": "交错双窗口墙"}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def wilson(success: int, count: int) -> tuple[float, float]:
    z = 1.959963984540054
    proportion = success / count
    scale = 1 + z * z / count
    center = (proportion + z * z / (2 * count)) / scale
    radius = z * math.sqrt(proportion * (1 - proportion) / count + z * z / (4 * count * count)) / scale
    return max(0.0, 100 * (center - radius)), min(100.0, 100 * (center + radius))


def successful_metrics(rows: list[dict[str, str]]) -> dict:
    good = [row for row in rows if row["success"] == "1"]
    result = {"population": "successful trials only", "count": len(good)}
    for field in ("time", "path", "formationRms", "verticalTravel", "wallSeconds"):
        values = [float(row[field]) for row in good]
        assert all(math.isfinite(value) for value in values)
        result[field] = {"mean": sum(values) / len(values), "min": min(values), "max": max(values)} if values else None
    return result


def analyze(study: Path, pilot: Path | None) -> dict:
    protocol = json.loads((study / "protocol.json").read_text(encoding="utf-8-sig"))
    assert protocol["p"]["version"] == "2.1.0"
    assert protocol["seeds"] == list(range(1, 1001))
    assert protocol["p"]["heightLayers"] == [5, 10, 15, 20, 25]
    assert protocol["heightPolicy"] == "fixed_5_10_15_20_25_no_insertion"
    assert protocol["p"]["verticalCostWeight"] == 1
    statuses, rows, raw_files, error_files = [], [], [], []
    for worker, (start, stop) in enumerate(protocol["ranges"], 1):
        folder = study / "workers" / f"worker_{worker:02d}"
        status = json.loads((folder / "status.json").read_text(encoding="utf-8-sig"))
        assert status["worker"] == worker and status["status"] == "complete"
        assert status["completed"] == status["scheduled"] == stop - start + 1
        worker_rows = read_csv(folder / "trials.csv")
        assert [int(row["seed"]) for row in worker_rows] == list(range(start, stop + 1))
        assert status["success"] == sum(row["success"] == "1" for row in worker_rows)
        statuses.append(status)
        rows.extend(worker_rows)
        raw_files.extend((folder / "trials").glob("*.mat"))
        error_files.extend((folder / "trials").glob("*.error.txt"))
    rows.sort(key=lambda row: int(row["seed"]))
    assert len(rows) == len({row["trialId"] for row in rows}) == len({row["seed"] for row in rows}) == 1000
    assert [int(row["seed"]) for row in rows] == list(range(1, 1001))
    assert len(raw_files) == 1000 and all(path.stat().st_size > 0 for path in raw_files)
    expected_files = {f"C_random_{seed:06d}.mat" for seed in range(1, 1001)}
    assert {path.name for path in raw_files} == expected_files
    for row in rows:
        seed = int(row["seed"])
        assert row["trialId"] == f"C_random_{seed:06d}"
        assert row["family"] == FAMILIES[(seed - 1) % 4]
        assert row["group"] == "C" and row["layerCount"] == "5" and row["verticalCostWeight"] == "1"
        assert row["success"] in ("0", "1")
        assert (row["success"] == "1") == (row["reason"] == "success")
        if int(row["cycles"]) == 0:
            assert all(math.isnan(float(row[name])) for name in ("path", "formationRms", "verticalTravel"))
    frozen = read_csv(study / "source_sha256.csv")
    assert len(frozen) == 25
    for source in frozen:
        assert sha256(study / "source_snapshot" / source["relativePath"]) == source["sha256"]
    audit = json.loads((study / "raw_mat_audit.json").read_text(encoding="utf-8-sig"))
    mat_rows = read_csv(study / "raw_mat_audit.csv")
    assert audit["passed"] and audit["records"] == audit["uniqueSeeds"] == 1000
    assert [int(row["seed"]) for row in mat_rows] == list(range(1, 1001))
    for row, raw in zip(rows, mat_rows):
        assert row["seed"] == raw["seed"] and row["family"] == raw["family"]
        assert row["reason"] == raw["reason"] and row["success"] == raw["success"]
        assert row["cycles"] == raw["cycles"]
    success = sum(row["success"] == "1" for row in rows)
    assert success == audit["success"]
    outcomes = dict(sorted(Counter(row["reason"] for row in rows).items()))
    assert sum(outcomes.values()) == 1000
    assert len(error_files) == outcomes.get("execution_error", 0)
    families, family_csv = {}, []
    for family in FAMILIES:
        subset = [row for row in rows if row["family"] == family]
        assert len(subset) == 250
        good = sum(row["success"] == "1" for row in subset)
        low, high = wilson(good, len(subset))
        reasons = dict(sorted(Counter(row["reason"] for row in subset).items()))
        missed = sum(int(row["gatesWithoutFixedLayer"]) > 0 for row in mat_rows if row["family"] == family)
        families[family] = {"label": LABELS[family], "scheduled": 250, "success": good,
            "failure": 250 - good, "successPercent": 100 * good / 250,
            "wilson95Percent": [low, high], "outcomes": reasons, "missedWindowSeeds": missed}
        family_csv.append({"family": family, "label": LABELS[family], "scheduled": 250,
            "success": good, "failure": 250 - good, "successPercent": 100 * good / 250,
            "wilson95LowerPercent": low, "wilson95UpperPercent": high,
            "missedWindowSeeds": missed})
    assert sum(group["success"] for group in families.values()) == success
    replication = []
    compatibility = []
    if pilot:
        pilot_sources = read_csv(pilot / "source_sha256.csv")
        lookup = {source["sourceFiles"].replace("\\", "/"): source["sha256"] for source in pilot_sources}
        for source in frozen:
            relative = source["relativePath"]
            if relative.startswith(("config/", "src3d/")):
                assert relative in lookup and lookup[relative] == source["sha256"]
                compatibility.append({"relativePath": relative, "pilotSha256": lookup[relative],
                    "sweepSha256": source["sha256"], "match": True})
        assert len(compatibility) == 23
        by_seed = {int(row["seed"]): row for row in read_csv(pilot / "trials.csv") if row["sceneId"] == "random"}
        fields = {"time": "time", "path": "leaderPath", "formationRms": "formationRms",
                  "minObstacleClearance": "minObstacleClearance", "minPairDistance": "minPairDistance"}
        for seed in range(1, 17):
            previous, current = by_seed[seed], rows[seed - 1]
            assert previous["reason"] == current["reason"] and previous["success"] == current["success"]
            for field, old_field in fields.items():
                a, b = float(current[field]), float(previous[old_field])
                assert (math.isnan(a) and math.isnan(b)) or math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9)
            replication.append({"seed": seed, "outcomeMatches": True, "numericalMetricsMatch": True})
        write_csv(study / "pilot_source_compatibility.csv", compatibility, list(compatibility[0]))
        write_csv(study / "pilot_replication.csv", replication, list(replication[0]))
    raw_manifest = [{"relativePath": path.relative_to(study).as_posix(), "bytes": path.stat().st_size,
                     "sha256": sha256(path)} for path in sorted(raw_files)]
    write_csv(study / "trials_sha256.csv", raw_manifest, ["relativePath", "bytes", "sha256"])
    write_csv(study / "trials_all_1000.csv", rows, list(rows[0]))
    write_csv(study / "failures.csv", [row for row in rows if row["success"] != "1"], list(rows[0]))
    write_csv(study / "family_summary.csv", family_csv, list(family_csv[0]))
    write_csv(study / "outcomes.csv", [{"reason": reason, "count": count, "percentOfAllSeeds": count / 10}
        for reason, count in outcomes.items()], ["reason", "count", "percentOfAllSeeds"])
    generated = sum(row["generationOK"] == "1" for row in rows)
    entered = sum(int(row["cycles"]) > 0 for row in rows)
    summary = {"version": "2.1.0", "firstSeed": 1, "lastSeed": 1000, "scheduled": 1000,
        "completed": 1000, "uniqueSeeds": 1000, "success": success, "failure": 1000 - success,
        "successPercent": success / 10, "denominator": "All predeclared seed 1--1000, no outcome exclusions",
        "fixedHeightLayersM": [5, 10, 15, 20, 25], "heightInsertionOrAdaptation": False,
        "verticalCostWeight": 1, "group": "C", "noiseStd": 0, "disturbanceMax": 0,
        "familyAllocation": "250 seeds per family, seed modulo four",
        "overallIntervalRule": "No iid pooled Wilson interval; family counts were fixed in advance",
        "outcomes": outcomes, "families": families, "generationOK": generated,
        "enteredFlightLoop": entered, "successAmongEnteredFlightsPercent": 100 * success / entered if entered else None,
        "cycles": sum(int(row["cycles"]) for row in rows),
        "seedsWithSafetyTriggers": sum(int(row["safetyTriggers"]) > 0 for row in rows),
        "safetyTriggerCycles": sum(int(row["safetyTriggers"]) for row in rows),
        "rawMatFiles": 1000, "rawMatBytes": sum(item["bytes"] for item in raw_manifest),
        "rawMatAudit": audit, "frozenSourceFiles": 25, "sourceHashMismatches": 0,
        "pilotNumericSourceMatches": len(compatibility), "pilotReplicatedSeeds": len(replication),
        "workers": statuses, "successfulTrialMetrics": successful_metrics(rows),
        "analyzedAt": datetime.now(timezone.utc).isoformat(),
        "timingCaveat": "Four concurrent MATLAB workers; these timings are not a realtime benchmark",
        "interpretationCaveat": "Equal mixture of four specified synthetic map families; not a real-world universal success rate"}
    (study / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    family_table = "\n".join(f"| {LABELS[family]} | 250 | {item['success']} | {item['failure']} | {item['successPercent']:.1f}% |"
        for family, item in families.items())
    failure_table = "\n".join(f"| {reason} | {count} |" for reason, count in outcomes.items() if reason != "success")
    if not failure_table:
        failure_table = "| 无失败 | 0 |"
    missed = audit["missedWindowSeeds"]
    text = f"""# 新障碍物 · 固定五个高度层 · 1000 seed 结果

## 结果

**成功 {success}/1000，成功率 {success / 10:.1f}%；失败 {1000 - success}/1000。**

执行完整、唯一的 seed 1–1000。生成失败、初始规划失败、飞行失败和执行错误均保留在分母中。前期 16 个 pilot seed 在本轮重跑，属于这 1000 个 seed，不额外累计。

| 障碍场景 | 总数 | 成功 | 失败 | 成功率 |
| --- | ---: | ---: | ---: | ---: |
{family_table}
| **合计** | **1000** | **{success}** | **{1000 - success}** | **{success / 10:.1f}%** |

## 失败结局

| 仿真结局 | 数量 |
| --- | ---: |
{failure_table}

有 **{missed}** 张窗口地图至少有一个横贯全宽的窗口，其 4.2 m 规划净空区间内没有任何固定高度层。该统计直接读取原始障碍几何，详见 `raw_mat_audit.csv`；它说明固定层图漏掉该结构窗口，**没有证明所有失败地图都能在连续空间安全飞通**。本轮没有补入高度层，也没有为失败 seed 另跑航迹。

## 实验口径

- 当前 2.1.0 新生成器，混合三维障碍、顶棚下穿、单窗口墙、交错双窗口墙各 250 个 seed，按 seed 模 4 分配，不按结局筛图。
- 规划层始终为 **5、10、15、20、25 m**。五层指图节点候选高度；飞行器连续运动，层间转移与简化航段仍做连续碰撞检查。
- 空间 80 × 80 × 30 m；水平网格 1 m；长机规划净空 4.2 m；每图请求 8 个障碍。
- 5 架无人机，默认 C 组（ADP 与安全层开启），每 seed 重置共享权重；位置噪声和外部加速度均为 0。
- 速度上限 3 m/s，加速度命令上限 3 m/s²，步长 0.05 s，任务时限 120 s。高度变化代价权重保持 1。
- 成功需长机目标误差 ≤1 m、僚机参考误差 ≤1 m、全队最大速度 ≤0.25 m/s，连续保持 1 s，并通过碰撞与边界检查。
- 该 {success / 10:.1f}% 对应四类地图等量混合的当前分布。不能直接当作真实飞行普遍成功率，也不能与旧 2.0.0 开放边缘地图的 100% 当作同一分布的算法对照。
- 四个独立 MATLAB 进程并行。本轮时序数据受并行负载影响，不作为实时性性能结论；本机即可完成，无需租算力。

## 数据核对

- 1000 行 CSV、1000 个唯一 seed、1000 个逐 seed MAT；四个 worker 全部完成。
- MATLAB 独立读取全部 MAT：初始参数与冻结协议完全一致，固定层未改变，CSV 字段与原始记录逐项匹配，成功任务的最终 1 s 和安全距离检查通过。
- 25/25 冻结源文件 SHA-256 匹配；与 2.1.0 pilot 的 {len(compatibility)}/23 数值相关源文件一致，前 {len(replication)} 个 seed 的结局和确定性数值指标复现一致。
- 飞行前失败的路径、队形、垂直运动指标保留 NaN，未用 0 伪装成有效测量。成功任务均值仅从成功任务计算。

## 文件与复现

原始结果目录：`{study}`。

- `trials_all_1000.csv`：全部 seed 结局和指标；`failures.csv`：所有失败记录；`family_summary.csv`：分场景统计和场景内 Wilson 95% 区间。
- `workers/worker_*/trials/*.mat`：地图、参数、控制与轨迹原始数据；`trials_sha256.csv`：逐 MAT 哈希。
- `protocol.mat/json`、`source_snapshot/`、`source_sha256.csv`：执行时冻结的协议与源码。
- `raw_mat_audit.csv/json`：全部原始文件的独立核对与窗口覆盖记录。
- `analysis_source.py`、`audit_source.m`：本次统计与核对脚本副本；`pilot_replication.csv`、`pilot_source_compatibility.csv`：历史小样本复现证据。

重新计算统计（不重新飞行）：先在 MATLAB 调用 `audit_new_map_seed_sweep3d(<结果目录>)`，再用 Python 执行 `analyze_new_map_seed_sweep3d.py <结果目录> --pilot <2.1.0 pilot 目录>`。重新飞行应复制本轮冻结协议与源码到新的结果目录，避免覆盖本轮 1000 个原始 MAT。
"""
    (study / "NEW_MAP_SEED_SWEEP_REPORT_20261005.md").write_text(text, encoding="utf-8")
    shutil.copyfile(Path(__file__), study / "analysis_source.py")
    shutil.copyfile(Path(__file__).with_name("audit_new_map_seed_sweep3d.m"), study / "audit_source.m")
    receipt_rows = [{"family": LABELS[family], "count": item["scheduled"],
        "success": item["success"], "failure": item["failure"], "success_percent": item["successPercent"]}
        for family, item in families.items()]
    receipt_rows.append({"family": "合计", "count": 1000, "success": success,
                         "failure": 1000 - success, "success_percent": success / 10})
    receipt = {"schemaVersion": 1, "items": [{"id": "fixed-five-layer-success",
        "title": "新障碍物下，固定五层的 1000 seed 成功率", "queries": [{
            "id": "all-seeds-by-family", "source": {"label": "2.1.0 固定五层实验",
                "files": [{"label": name} for name in ("trials_all_1000.csv", "protocol.json", "raw_mat_audit.csv", "source_sha256.csv")],
                "executedAt": summary["analyzedAt"],
                "metricDefinitions": [{"label": "任务成功率", "definition": "任务成功率涵盖预定 seed 1–1000，所有生成、规划和飞行失败都计入分母；四类场景各占 250 个 seed。"}],
                "filters": ["固定高度层：5、10、15、20、25 m", "控制配置：C 组", "位置噪声：0", "外部加速度：0"],
                "caveats": ["该成功率对应四类合成障碍地图的等量混合，不能视为真实飞行的普遍成功率。"],
                "evidenceFlow": [
                    {"kind": "validation", "title": "种子与原始记录覆盖", "detail": "四个 worker 全部完成；1000 个唯一 seed 与 1000 个原始 MAT 齐全，CSV 结局与 MATLAB 独立读取的原始结局逐项一致。"},
                    {"kind": "validation", "title": "固定配置", "detail": "全部原始初始参数与冻结协议一致，五层未改变；25 个冻结源文件的 SHA-256 全部匹配。"}]},
            "rows": receipt_rows,
            "columns": [{"field": "family", "label": "场景"}, {"field": "count", "label": "总数"},
                {"field": "success", "label": "成功"}, {"field": "failure", "label": "失败"},
                {"field": "success_percent", "label": "成功率（%）"}],
            "preview": {"kind": "aggregate", "note": "1000 个 seed 的场景分组计数和总计。", "totalRows": 1000},
            "methods": [{"language": "calculation", "code": f"总体：{success} / 1000 × 100 = {success / 10:.1f}%\n各场景成功率 = 各场景成功数 / 250 × 100"}]
        }]}]}
    (study / "sources_receipt.json").write_text(json.dumps(receipt, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("study", type=Path)
    parser.add_argument("--pilot", type=Path, required=True)
    arguments = parser.parse_args()
    result = analyze(arguments.study, arguments.pilot)
    print(json.dumps({key: result[key] for key in ("success", "failure", "successPercent", "outcomes", "families", "rawMatFiles")}, ensure_ascii=False, indent=2))

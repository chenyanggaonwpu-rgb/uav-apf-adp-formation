"""Summarize a completed 3-D study from its trial-level CSV (stdlib only).

Usage: python experiments3d/analyze_study3d.py results/3d_study_... 
The output is written beside trials.csv; the original rows are never changed.
"""

from __future__ import annotations

import csv
import json
import math
import random
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path


DT = 0.05
DIRECT_DISTANCE_M = math.sqrt(60**2 + 52**2)
BOOTSTRAP_SEED = 20261003
BOOTSTRAP_REPETITIONS = 4000
METRICS = ("time", "path", "formationRms", "cycleMeanMs")


def value(row: dict[str, str], key: str) -> float:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return math.nan


def finite(values: list[float]) -> list[float]:
    return [v for v in values if math.isfinite(v)]


def mean(values: list[float]) -> float | None:
    valid = finite(values)
    return statistics.mean(valid) if valid else None


def percentile(sorted_values: list[float], percent: float) -> float:
    index = (len(sorted_values) - 1) * percent
    lower = math.floor(index)
    upper = math.ceil(index)
    return sorted_values[lower] + (index - lower) * (
        sorted_values[upper] - sorted_values[lower]
    )


def bootstrap_mean_interval(values: list[float]) -> list[float] | None:
    if not values:
        return None
    if len(values) == 1:
        return [values[0], values[0]]
    rng = random.Random(BOOTSTRAP_SEED)
    n = len(values)
    estimates = sorted(
        statistics.mean(values[rng.randrange(n)] for _ in range(n))
        for _ in range(BOOTSTRAP_REPETITIONS)
    )
    return [percentile(estimates, 0.025), percentile(estimates, 0.975)]


def wilson95(successes: int, total: int) -> list[float] | None:
    if total == 0:
        return None
    z = 1.959963984540054
    rate = successes / total
    denominator = 1 + z * z / total
    center = (rate + z * z / (2 * total)) / denominator
    radius = z * math.sqrt(
        rate * (1 - rate) / total + z * z / (4 * total * total)
    ) / denominator
    return [center - radius, center + radius]


def group_summary(rows: list[dict[str, str]]) -> dict:
    successful = [r for r in rows if r["reason"] == "success"]
    random_rows = [r for r in rows if r["mapKind"] == "random"]
    fixed_rows = [r for r in rows if r["mapKind"] == "fixed"]
    random_success = sum(r["reason"] == "success" for r in random_rows)
    return {
        "tasks": len(rows),
        "outcomes": dict(Counter(r["reason"] for r in rows)),
        "fixed_tasks": len(fixed_rows),
        "fixed_successes": sum(r["reason"] == "success" for r in fixed_rows),
        "random_tasks": len(random_rows),
        "random_successes": random_success,
        "random_wilson95": wilson95(random_success, len(random_rows)),
        "successful_mean_time_s": mean([value(r, "time") for r in successful]),
        "successful_mean_path_m": mean([value(r, "path") for r in successful]),
        "successful_mean_path_coefficient": mean(
            [value(r, "path") / DIRECT_DISTANCE_M for r in successful]
        ),
        "successful_mean_formation_rms_m": mean(
            [value(r, "formationRms") for r in successful]
        ),
        "mean_trial_cycle_mean_ms": mean([value(r, "cycleMeanMs") for r in rows]),
        "max_trial_cycle_p99_ms": max(
            finite([value(r, "cycleP99Ms") for r in rows]), default=None
        ),
        "max_recorded_cycle_ms": max(
            finite([value(r, "cycleMaxMs") for r in rows]), default=None
        ),
        "min_obstacle_clearance_m": min(
            finite([value(r, "minObstacleClearance") for r in rows]),
            default=None,
        ),
        "min_pair_distance_m": min(
            finite([value(r, "minPairDistance") for r in rows]), default=None
        ),
        "safety_triggers": sum(
            round(value(r, "safetyTriggers"))
            for r in rows
            if math.isfinite(value(r, "safetyTriggers"))
        ),
    }


def paired_summary(rows: list[dict[str, str]], first: str, second: str) -> dict:
    maps: dict[tuple[str, str, str], dict[str, dict[str, str]]] = defaultdict(dict)
    for row in rows:
        maps[(row["mapKind"], row["mapId"], row["seed"])][row["group"]] = row
    both = [
        (pair[first], pair[second])
        for pair in maps.values()
        if first in pair
        and second in pair
        and pair[first]["reason"] == "success"
        and pair[second]["reason"] == "success"
    ]
    summary = {"common_success_maps": len(both)}
    for metric in METRICS:
        differences = finite([value(b, metric) - value(a, metric) for a, b in both])
        summary[f"mean_{metric}_difference"] = mean(differences)
        summary[f"bootstrap95_mean_{metric}_difference"] = (
            bootstrap_mean_interval(differences)
        )
    return summary


def timing_summary(rows: list[dict[str, str]]) -> dict:
    full_cycles = [
        round(value(r, "time") / DT)
        for r in rows
        if math.isfinite(value(r, "time")) and math.isfinite(value(r, "cycleMeanMs"))
    ]
    late_cycles = [
        round(value(r, "over50Fraction") * round(value(r, "time") / DT))
        for r in rows
        if math.isfinite(value(r, "time"))
        and math.isfinite(value(r, "over50Fraction"))
    ]
    result = {
        "total_recorded_cycles": sum(full_cycles),
        "cycles_over_50_ms": sum(late_cycles),
        "fraction_over_50_ms": (
            sum(late_cycles) / sum(full_cycles) if full_cycles else None
        ),
        "trials_with_over_50_ms": [
            r["trialId"] for r in rows if value(r, "over50Fraction") > 0
        ],
        "max_trial_p99_ms": max(
            finite([value(r, "cycleP99Ms") for r in rows]), default=None
        ),
        "max_recorded_cycle_ms": max(
            finite([value(r, "cycleMaxMs") for r in rows]), default=None
        ),
        "median_initial_plan_s": statistics.median(
            finite([value(r, "initialPlanSeconds") for r in rows])
        ),
        "max_initial_plan_s": max(
            finite([value(r, "initialPlanSeconds") for r in rows]), default=None
        ),
        "total_replan_events": sum(
            round(value(r, "replanCount"))
            for r in rows
            if math.isfinite(value(r, "replanCount"))
        ),
        "max_replan_s": max(
            finite([value(r, "maxReplanSeconds") for r in rows]), default=None
        ),
        "total_terminal_plan_events": sum(
            round(value(r, "terminalPlanCount"))
            for r in rows
            if math.isfinite(value(r, "terminalPlanCount"))
        ),
        "max_terminal_plan_s": max(
            finite([value(r, "maxTerminalPlanSeconds") for r in rows]),
            default=None,
        ),
    }
    return result


def analyze(directory: Path) -> dict:
    with (directory / "trials.csv").open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    ids = [r["trialId"] for r in rows]
    if len(ids) != len(set(ids)):
        raise ValueError("Duplicate trialId in trials.csv")
    groups = {
        group: [r for r in rows if r["trialId"].startswith(group + "_")]
        for group in ("A", "B", "C")
    }
    perturbation = [
        {
            "trial": r["trialId"],
            "reason": r["reason"],
            "time_s": value(r, "time"),
            "path_m": value(r, "path"),
            "path_coefficient": value(r, "path") / DIRECT_DISTANCE_M,
            "formation_rms_m": value(r, "formationRms"),
            "min_obstacle_m": value(r, "minObstacleClearance"),
            "min_pair_m": value(r, "minPairDistance"),
            "safety_triggers": value(r, "safetyTriggers"),
        }
        for r in rows
        if r["trialId"].startswith(("noise_", "disturbance_"))
    ]
    return {
        "trial_count": len(rows),
        "unique_trial_ids": len(set(ids)),
        "outcomes": dict(Counter(r["reason"] for r in rows)),
        "groups": {group: group_summary(part) for group, part in groups.items()},
        "paired": {
            "B_minus_A": paired_summary(
                [r for part in groups.values() for r in part], "A", "B"
            ),
            "C_minus_B": paired_summary(
                [r for part in groups.values() for r in part], "B", "C"
            ),
        },
        "timing": timing_summary(rows),
        "perturbation": perturbation,
        "bootstrap_seed": BOOTSTRAP_SEED,
        "bootstrap_repetitions": BOOTSTRAP_REPETITIONS,
    }


if __name__ == "__main__":
    folder = Path(sys.argv[1])
    summary = analyze(folder)
    output = folder / "analysis_summary.json"
    output.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(f"{summary['trial_count']} trials; {summary['outcomes']}; {output}")

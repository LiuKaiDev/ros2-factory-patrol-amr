"""Deterministic statistics and serialization for Factory Patrol benchmarks."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


def finite_samples(values: Iterable[float]) -> list[float]:
    samples = [float(value) for value in values]
    if any(not math.isfinite(value) for value in samples):
        raise ValueError("metric samples must all be finite")
    return samples


def percentile_nearest_rank(values: Iterable[float], percentile: float) -> float:
    """Return a nearest-rank percentile (rank=ceil(p/100*n), one based)."""
    samples = sorted(finite_samples(values))
    if not samples:
        raise ValueError("percentile requires at least one sample")
    if not math.isfinite(percentile) or percentile <= 0.0 or percentile > 100.0:
        raise ValueError("percentile must be in (0, 100]")
    rank = max(1, math.ceil(percentile / 100.0 * len(samples)))
    return samples[rank - 1]


def summarize(values: Iterable[float], *, include_rmse: bool = False) -> dict[str, float | int]:
    samples = finite_samples(values)
    if not samples:
        raise ValueError("summary requires at least one sample")
    count = len(samples)
    mean = sum(samples) / count
    ordered = sorted(samples)
    middle = count // 2
    median = (
        ordered[middle]
        if count % 2
        else (ordered[middle - 1] + ordered[middle]) / 2.0
    )
    variance = sum((value - mean) ** 2 for value in samples) / count
    result: dict[str, float | int] = {
        "count": count,
        "min": min(samples),
        "max": max(samples),
        "mean": mean,
        "median": median,
        "p50": percentile_nearest_rank(samples, 50.0),
        "p95": percentile_nearest_rank(samples, 95.0),
        "stddev": math.sqrt(variance),
    }
    if include_rmse:
        result["rmse"] = math.sqrt(sum(value * value for value in samples) / count)
    return result


def summarize_xyz(points: Sequence[Sequence[float]]) -> dict[str, Any]:
    if not points:
        raise ValueError("position summary requires at least one point")
    if any(len(point) != 3 for point in points):
        raise ValueError("positions must contain exactly x, y, and z")
    axes = list(zip(*points))
    return {
        "count": len(points),
        "x": summarize(axes[0]),
        "y": summarize(axes[1]),
        "z": summarize(axes[2]),
    }


def false_task_trigger_count(mission_starts: int, physical_targets: int) -> int:
    """Count mission starts beyond the one intended start per physical target."""
    if (
        isinstance(mission_starts, bool)
        or isinstance(physical_targets, bool)
        or not isinstance(mission_starts, int)
        or not isinstance(physical_targets, int)
        or mission_starts < 0
        or physical_targets < 0
    ):
        raise ValueError("mission starts and physical targets must be nonnegative integers")
    return max(0, mission_starts - physical_targets)


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def write_json(path: str | Path, result: Mapping[str, Any]) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n"
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(payload, encoding="utf-8")
    temporary.replace(target)


@dataclass(frozen=True)
class PerformanceRow:
    metric: str
    samples: int | str
    mean: float | str
    p50: float | str
    p95: float | str
    maximum: float | str
    unit: str


def write_performance_csv(path: str | Path, rows: Sequence[PerformanceRow]) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["metric", "sample_count", "mean", "p50", "p95", "max", "unit"])
        for row in rows:
            writer.writerow(
                [row.metric, row.samples, row.mean, row.p50, row.p95, row.maximum, row.unit]
            )
    temporary.replace(target)

import json
import math

import pytest

from robot_experiments.benchmark_metrics import (
    PerformanceRow,
    false_task_trigger_count,
    percentile_nearest_rank,
    summarize,
    summarize_xyz,
    write_json,
    write_performance_csv,
)


def test_summary_uses_population_standard_deviation_and_nearest_rank():
    result = summarize([1.0, 2.0, 3.0, 4.0], include_rmse=True)
    assert result["count"] == 4
    assert result["mean"] == pytest.approx(2.5)
    assert result["median"] == pytest.approx(2.5)
    assert result["p50"] == pytest.approx(2.0)
    assert result["p95"] == pytest.approx(4.0)
    assert result["stddev"] == pytest.approx(math.sqrt(1.25))
    assert result["rmse"] == pytest.approx(math.sqrt(7.5))


def test_nearest_rank_boundaries():
    samples = list(range(1, 101))
    assert percentile_nearest_rank(samples, 50.0) == 50
    assert percentile_nearest_rank(samples, 95.0) == 95
    assert percentile_nearest_rank([7.0], 95.0) == 7.0


@pytest.mark.parametrize("values", [[], [1.0, math.nan], [math.inf]])
def test_summary_rejects_empty_or_invalid_samples(values):
    with pytest.raises(ValueError):
        summarize(values)


def test_position_summary_validates_shape():
    result = summarize_xyz([(1.0, 2.0, 3.0), (3.0, 4.0, 5.0)])
    assert result["count"] == 2
    assert result["x"]["mean"] == pytest.approx(2.0)
    with pytest.raises(ValueError):
        summarize_xyz([(1.0, 2.0)])


def test_false_task_triggers_count_only_extra_mission_starts():
    assert false_task_trigger_count(1, 1) == 0
    assert false_task_trigger_count(2, 1) == 1
    assert false_task_trigger_count(0, 1) == 0


@pytest.mark.parametrize("values", [(-1, 1), (1, -1), (1.5, 1), (True, 1)])
def test_false_task_triggers_reject_invalid_counts(values):
    with pytest.raises(ValueError):
        false_task_trigger_count(*values)


def test_result_serialization_is_strict_and_atomic(tmp_path):
    output = tmp_path / "result.json"
    write_json(output, {"valid": True, "metric": 1.25})
    assert json.loads(output.read_text(encoding="utf-8")) == {
        "metric": 1.25,
        "valid": True,
    }
    with pytest.raises(ValueError):
        write_json(output, {"invalid": math.nan})


def test_performance_csv_serialization(tmp_path):
    output = tmp_path / "summary.csv"
    write_performance_csv(
        output,
        [PerformanceRow("latency", 3, 1.0, 1.0, 2.0, 2.0, "ms")],
    )
    text = output.read_text(encoding="utf-8")
    assert text.startswith("metric,sample_count,mean,p50,p95,max,unit\n")
    assert "latency,3,1.0,1.0,2.0,2.0,ms" in text

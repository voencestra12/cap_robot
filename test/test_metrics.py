"""역할: 정량 지표 분모·중복·미측정 회귀. 인터페이스: CsvMetrics/summarize."""
import pytest
from cap_robot.metrics import CsvMetrics, make_event
from cap_robot.metrics_report import summarize


def test_failures_counted_and_no_human_scores_invented(tmp_path):
    path = tmp_path / 'metrics.csv'
    csv = CsvMetrics(path)
    ok = make_event('gate', 'gate_command_result', success=True, cooperative=False)
    csv.write(ok)
    csv.write(ok)
    csv.write(make_event('gate', 'gate_command_result', success=False, cooperative=False))
    csv.close()
    result = summarize([path])
    assert result['actuator_command_samples'] == 2
    assert result['actuator_command_success_rate'] == .5
    assert result['command_recognition_rate'] is None
    assert result['human_prompt_compliance_rate'] is None


def test_latest_human_grade_per_request(tmp_path):
    path = tmp_path / 'metrics.csv'
    csv = CsvMetrics(path)
    csv.write(make_event('operator', 'human_evaluation', request_id='r', recognized=False))
    csv.write(make_event('operator', 'human_evaluation', request_id='r', recognized=True))
    csv.close()
    result = summarize([path])
    assert result['command_recognition_samples'] == 1
    assert result['command_recognition_rate'] == 1.

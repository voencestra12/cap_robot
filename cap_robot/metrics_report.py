"""역할: CSV 정량 지표 집계. 인터페이스: ros2 run cap_robot metrics_report <CSV...>.

# [변경] 의미 인식률은 사람이 채점한 표본만 분모로 사용하고 미측정은 null로 표기.
"""

import argparse
import csv
import json
import statistics
import math


def summarize(paths):
    seen = set()
    events = []
    for path in paths:
        with open(path, encoding="utf-8", newline="") as f:
            for row in csv.DictReader(f):
                if row["event_id"] in seen:
                    continue
                seen.add(row["event_id"])
                extra = json.loads(row.get("extra_json") or "{}")
                events.append(dict(row, **extra))

    def truth(v):
        return v is True or v == "True"

    def rate(rows, key):
        return sum(truth(r.get(key)) for r in rows) / len(rows) if rows else None

    inference = [r for r in events if r["event"] == "llm_inference"]
    latency = sorted(
        float(r["latency_s"]) for r in inference if r.get("latency_s") not in ("", None)
    )
    # Latest manual grade per request; repeated re-grading is not another experiment.
    grades = {
        r.get("request_id", r["event_id"]): r for r in events if r["event"] == "human_evaluation"
    }
    human = list(grades.values())
    recognized = [r for r in human if r.get("recognized") in (True, False)]
    compliant = [r for r in human if r.get("prompt_compliant") in (True, False)]
    complete = [r for r in human if r.get("task_completed") in (True, False)]
    commands = [r for r in events if r["event"] == "gate_command_result"]
    single = [r for r in commands if not r.get('cooperative',False)]
    return dict(
        inference_attempts=len(inference),
        inference_latency_mean_s=statistics.mean(latency) if latency else None,
        inference_latency_p95_s=(
            latency[max(0, math.ceil(0.95 * len(latency)) - 1)] if latency else None
        ),
        structured_response_acceptance_rate=rate(inference, "success"),
        command_recognition_rate=rate(recognized, "recognized"),
        command_recognition_samples=len(recognized),
        human_prompt_compliance_rate=rate(compliant, "prompt_compliant"),
        human_prompt_compliance_samples=len(compliant),
        human_task_success_rate=rate(complete, "task_completed"),
        human_task_success_samples=len(complete),
        actuator_command_success_rate=rate(commands, "success"),
        actuator_command_samples=len(commands),
        single_agent_command_success_rate=rate(single, "success"),
        single_agent_command_samples=len(single),
        replan_events=sum(r["event"] == "replan_trigger" for r in events),
        cooperative_steps=sum(r["event"] == "cooperative_step" for r in events),
    )


def main(args=None):
    p = argparse.ArgumentParser(description="Cap robot CSV metrics; null means unmeasured")
    p.add_argument("csv", nargs="+")
    opt = p.parse_args(args)
    print(json.dumps(summarize(opt.csv), ensure_ascii=False, indent=2, allow_nan=False))

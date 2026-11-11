"""Reproduce the original four-domain matrix with isolated, bounded processes."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def worker(problem, search, heuristic):
    from pyperplan.planner import SEARCHES, HEURISTICS, _parse, _ground, find_domain
    start = time.monotonic()
    task = _ground(_parse(find_domain(problem), problem))
    h = HEURISTICS[heuristic](task)
    search_start = time.monotonic()
    plan = SEARCHES[search](task, h)
    elapsed = time.monotonic() - search_start
    if plan is not None:
        state = task.initial_state
        for op in plan:
            if not op.applicable(state):
                raise AssertionError("Invalid plan precondition")
            state = op.apply(state)
        assert task.goal_reached(state)
    return dict(status="solved" if plan is not None else "no_plan",
                length=len(plan) if plan is not None else None,
                search_seconds=elapsed, total_seconds=time.monotonic()-start,
                heuristic_calls=h.get_call_counter(),
                refinements=getattr(h, "version", 0),
                counter_growth=getattr(h, "counter_growth", None))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", nargs=3, metavar=("PROBLEM", "SEARCH", "HEURISTIC"))
    parser.add_argument("--limit", type=int, default=0, help="Tasks per domain; 0 means all")
    parser.add_argument("--timeout", type=float, default=120)
    parser.add_argument("--output", default="benchmark_data/comparison.json")
    args = parser.parse_args()
    if args.worker:
        print(json.dumps(worker(*args.worker)))
        return
    root = Path(__file__).resolve().parent
    configs = [(s, h) for s in ["ehc", "eehc", "gehc", "hb_ehc", "hybrid_ehc"]
               for h in ["hff", "hcff"]] + [("rhc", "hcff")]
    results = []
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    for domain in ["rovers", "depot", "miconic", "satellite"]:
        tasks = sorted((root / "benchmarks" / domain).glob("task*.pddl"))
        if args.limit:
            tasks = tasks[:args.limit]
        for task in tasks:
            for search, heuristic in configs:
                row = dict(domain=domain, task=task.name, search=search, heuristic=heuristic)
                start = time.monotonic()
                try:
                    run = subprocess.run(
                        [sys.executable, str(Path(__file__).resolve()), "--worker",
                         str(task), search, heuristic], cwd=root,
                        env=dict(os.environ, PYTHONHASHSEED="0"),
                        capture_output=True, text=True, timeout=args.timeout)
                    if run.returncode:
                        row.update(status="error", error=run.stderr[-4000:])
                    else:
                        row.update(json.loads(run.stdout))
                except subprocess.TimeoutExpired:
                    row.update(status="timeout")
                row["wall_seconds"] = time.monotonic() - start
                results.append(row)
                output.write_text(json.dumps(dict(timeout=args.timeout, seed=0,
                    python=sys.version, results=results), indent=2))
                print(domain, task.name, search, heuristic, row["status"], flush=True)


if __name__ == "__main__":
    main()

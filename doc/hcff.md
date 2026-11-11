# Explicit-conjunction FF

`hcff` is a deterministic implementation of the direct explicit-conjunction
FF construction with **hC max-cost supporters**, as described by Hoffmann and
Fickert (2015), Definition 3 and greedy subgoal selection:
https://fai.cs.uni-saarland.de/hoffmann/papers/icaps15b-tr.pdf

It is non-admissible. Singleton-only values can differ from the existing
`hff`, which uses additive supporters. Conjunctions remain internal to the
heuristic; search states and operators retain real STRIPS semantics.

## Usage

```sh
.venv/bin/python -m pyperplan -H hcff -s gbf benchmarks/rovers/task01.pddl
.venv/bin/python -m pyperplan -H hcff -s rhc benchmarks/rovers/task01.pddl
```

All five custom EHC searches also accept `-H hcff`. Fixed conjunctions can be
provided with `--conjunctions file.json`: a JSON list of nonempty lists of
**exact grounded fact names**. All singletons are included automatically.

The Python API is `hCFFHeuristic(task, conjunctions=())`. `evaluate(node)`
returns the value, sequential operator occurrences, supported conjunctions,
prerequisites, dependency edges, and conjunction-set version. `__call__`
returns only the value. `calc_h_with_plan` preserves the existing preferred
operator API's set-of-names convention; use `evaluate` for ordered occurrences.

`add_conjunction` rebuilds all counters, including affected old counters,
and increments the version. `refine(node)` performs one conservative
**suffix-regression refinement** of the sequential relaxed plan. This policy
is not the Keyder/Fickert best-supporter-graph conflict ranking algorithm.
Repeated calls provide offline refinement. False means no new conjunction
was found by this policy, not a proof of unsolvability.

`rhc` performs depth-3 bounded BFS lookahead, refines when stuck, discards the
old lookahead, and retains conjunctions across restarts. After 100 refinements
or when this policy cannot refine, it falls back to GBFS with fixed C. It has
a cooperative 240-second search timeout. Python callers can configure the
horizon, refinement limit, and time limit. Use a process timeout for a hard
limit, including grounding and individual heuristic evaluations.

## Construction and validation

Counters propagate max distances for action/conjunction regressions. Extraction
regresses the highest-distance undominated subgoal, greedily merging compatible
subgoals only if every conjunction in their combined regression has a strictly
smaller distance. This includes cross-context conjunctions. Action occurrences
are retained separately; action names are not used to deduplicate the plan.
The graph handles overlapping add/delete effects using the repository's
add-wins operator semantics.

Tests cover initialization, counter reset, real delete conflicts, conjunction
refinement, full-conjunction real-plan extraction, all search integrations,
and generated tiny problems checked against exhaustive BFS. Full powersets
are only used on tiny test problems.

## Benchmarks

```sh
# Smoke matrix: 44 isolated runs, one task from each original domain.
.venv/bin/python benchmark_compare.py --limit 1 --timeout 20 --output benchmark_data/hcff-smoke.json
# Full original domain set, original searches with both heuristics, plus RHC.
.venv/bin/python benchmark_compare.py --timeout 120 --output benchmark_data/hcff-comparison.json
```

The runner fixes PYTHONHASHSEED=0, runs sequentially in isolated subprocesses,
enforces per-process wall-time limits, distinguishes errors/timeouts/no-plan,
replays all returned plans, and saves results after each run. It records plan
length, search and total time, heuristic calls, refinement count, and counter
growth. The five fixed-C hcff configurations start with singletons; RHC learns
conjunctions online. Benchmark this distinction explicitly when reporting results.

The checked-in smoke results are validation only, not a performance comparison
on the full benchmark set. All 44 smoke runs solved and replayed successfully.

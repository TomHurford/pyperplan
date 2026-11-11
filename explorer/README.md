# Search Observatory

Run from the repository root (no new runtime dependencies):

```sh
.venv/bin/python -m explorer.server
```

Open http://127.0.0.1:8765. Use `--port 9000` to choose another port.

Choose a benchmark domain, problem, search, and heuristic, then run. The graph
shows actual search-node creation, state expansions, and heuristic evaluations
from an isolated planner process. The live view follows incoming events. Pause,
step with the arrows, scrub the timeline, or replay at an adjustable event rate.
Space toggles playback; left/right arrows step; Home/End seek. Drag the graph to
pan, scroll to zoom, and use Fit to see the whole graph. Select nodes to inspect
fact changes and CFF relaxed plans. Select solution actions to inspect real
states. The heuristic chart also supports click-to-seek. Pin a summary before
running a second configuration, or export the full trace as JSON.

The notebook records completed lookaheads, backtracking/restarts, and learned
conjunctions. Refinement search currently uses the suffix-regression policy
documented in ../doc/hcff.md. Ordinary hcff starts with singleton conjunctions;
only rhc learns conjunctions during a run. CFF relaxed plans may ignore deletes
and should not be confused with the final, replay-validated real solution.

Playback controls the visualisation, not the running computation. Runs are
limited to 6,000 events and 60 seconds and can be cancelled. This explorer is
for inspecting individual problems, not unbiased timing comparisons. Use
benchmark_compare.py for performance experiments. Only one computation is
active per server; starting another replaces it. Trace data is retained in
memory and discarded when replaced or when the server exits.

Generated nodes are search-tree occurrences, not unique states. A hollow centre
marks a duplicate state. Expansion events are state-level: when multiple nodes
share a state, the first occurrence is highlighted. Generated/expanded colours
do not claim to represent the algorithm's exact open queue or pruning reasons.
The chart is heuristic value by evaluation order, not elapsed time. Facts marked
with a diamond are goals; green/red facts are added/deleted relative to the
parent. JSON exports preserve the full sequence for further analysis.

The server binds only to loopback. Static assets and the planner use standard
library Python; web fonts are optional and fall back to system fonts offline.

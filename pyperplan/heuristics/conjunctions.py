"""Explicit-conjunction relaxation, plan extraction, and suffix refinement.
"""

from collections import deque, namedtuple

from .heuristic_base import Heuristic


Counter = namedtuple("Counter", "operator conjunction regression prerequisites")


class ConjunctionGraph:
    """Compile a grounded task and its atomic conjunctions into counters."""

    def __init__(self, task, conjunctions=()):
        self.task = task
        facts = frozenset(task.facts)
        selected = {frozenset(c) for c in conjunctions}
        if any(not c or not c <= facts for c in selected):
            raise ValueError("Conjunctions must be nonempty subsets of task facts")
        selected.update(frozenset((fact,)) for fact in facts)
        self.conjunctions = frozenset(selected)
        self.non_singletons = tuple(c for c in self.conjunctions if len(c) > 1)
        self.counters = []
        self.dependents = {c: [] for c in self.conjunctions}
        self.achievers = {c: [] for c in self.conjunctions}
        for operator in task.operators:
            for conjunction in sorted(self.conjunctions, key=lambda c: tuple(sorted(c))):
                if not (operator.add_effects & conjunction):
                    continue
                if (operator.del_effects - operator.add_effects) & conjunction:
                    continue
                regression = operator.preconditions | (conjunction - operator.add_effects)
                prerequisites = self.contained_in(regression)
                counter = Counter(operator, conjunction, regression, prerequisites)
                self.counters.append(counter)
                self.achievers[conjunction].append(counter)
                for precondition in prerequisites:
                    self.dependents[precondition].append(counter)

    def contained_in(self, facts):
        """Return atomic conjunctions contained in a state or fact set."""
        return frozenset(frozenset((f,)) for f in facts if f in self.task.facts) | frozenset(
            c for c in self.non_singletons if c <= facts
        )

    def reachable(self, state):
        """Compute relaxed reachable conjunctions using fresh counter values.

        This is a reachability pass only; it does not extract an hCFF plan.
        """
        reached = set(self.contained_in(state))
        queue = deque(reached)
        remaining = [len(counter.prerequisites) for counter in self.counters]
        indices = {id(counter): i for i, counter in enumerate(self.counters)}
        for counter, count in zip(self.counters, remaining):
            if count == 0 and counter.conjunction not in reached:
                reached.add(counter.conjunction)
                queue.append(counter.conjunction)
        while queue:
            conjunction = queue.popleft()
            for counter in self.dependents[conjunction]:
                index = indices[id(counter)]
                remaining[index] -= 1
                if remaining[index] == 0 and counter.conjunction not in reached:
                    reached.add(counter.conjunction)
                    queue.append(counter.conjunction)
        return frozenset(reached)


# Occurrences retain their supported conjunctions, including repeated operators.
Occurrence = namedtuple("Occurrence", "operator supported prerequisites")
Evaluation = namedtuple("Evaluation", "value plan occurrences supporters version")


class hCFFHeuristic(Heuristic):
    """Explicit-conjunction FF using hC (max) supporters and greedy merging.

    Implements the direct, compilation-free construction of Hoffmann and
    Fickert (ICAPS 2015). Ties are deterministic. This is a satisficing,
    non-admissible heuristic; C defaults to the singleton conjunctions.
    """

    name = "hcff"

    def __init__(self, task, conjunctions=()):
        self.task = task
        self.graph = ConjunctionGraph(task, conjunctions)
        self.version = 0
        self.call_counter = 0
        self.baseline_counters = sum(len(op.add_effects) for op in task.operators)

    @property
    def counter_growth(self):
        return len(self.graph.counters) / max(1, self.baseline_counters)

    def add_conjunction(self, conjunction):
        conjunction = frozenset(conjunction)
        if conjunction in self.graph.conjunctions:
            return False
        # Build before mutation so invalid inputs leave the heuristic intact.
        graph = ConjunctionGraph(self.task, self.graph.conjunctions | {conjunction})
        self.graph = graph
        self.version += 1
        return True

    def _propagate(self, state):
        import heapq
        from itertools import count

        graph = self.graph
        distances = {c: float("inf") for c in graph.conjunctions}
        supporters = {}
        heap = []
        serial = count()
        remaining = [len(c.prerequisites) for c in graph.counters]
        indices = {id(c): i for i, c in enumerate(graph.counters)}

        def offer(c, value, supporter):
            if value < distances[c]:
                distances[c] = value
                supporters[c] = supporter
                heapq.heappush(heap, (value, next(serial), c))

        for c in sorted(graph.contained_in(state), key=lambda c: tuple(sorted(c))):
            offer(c, 0, None)
        for counter in graph.counters:
            if not counter.prerequisites:
                offer(counter.conjunction, 1, counter)
        settled = set()
        while heap:
            value, _, c = heapq.heappop(heap)
            if c in settled:
                continue
            settled.add(c)
            for counter in graph.dependents[c]:
                index = indices[id(counter)]
                remaining[index] -= 1
                if remaining[index] == 0:
                    # The last settled prerequisite has the maximum distance.
                    offer(counter.conjunction, value + 1, counter)
        return distances, supporters

    def evaluate(self, node):
        self.call_counter += 1
        graph = self.graph
        distances, best = self._propagate(node.state)
        pending = set(graph.contained_in(self.task.goals))
        if any(distances[c] == float("inf") for c in pending):
            return Evaluation(float("inf"), None, (), (), self.version)
        backwards = []
        while pending:
            pending = {c for c in pending if not c <= node.state}
            pending = {c for c in pending if not any(c < other for other in pending)}
            if not pending:
                break
            target = min(pending, key=lambda c: (-distances[c], tuple(sorted(c))))
            counter = best[target]
            op = counter.operator
            selected = {target}
            regression = counter.regression
            prerequisites = counter.prerequisites
            # Merge only when the combined context strictly precedes every
            # supported conjunction. Cross-context conjunctions are included.
            for candidate in sorted(pending - selected, key=lambda c: tuple(sorted(c))):
                if not op.add_effects & candidate or (op.del_effects - op.add_effects) & candidate:
                    continue
                combined = regression | op.preconditions | (candidate - op.add_effects)
                required = graph.contained_in(combined)
                level = max((distances[c] for c in required), default=0)
                if level < min(distances[c] for c in selected | {candidate}):
                    selected.add(candidate)
                    regression, prerequisites = combined, required
            supported = frozenset(c for c in graph.conjunctions
                                  if any(c <= chosen for chosen in selected)
                                  and op.add_effects & c)
            backwards.append(Occurrence(op, supported, prerequisites))
            pending.difference_update(selected)
            pending.update(prerequisites)
        occurrences = tuple(reversed(backwards))
        # Dependency edges: (producer occurrence, consumer occurrence, conjunction).
        achieved = {c: None for c in graph.contained_in(node.state)}
        edges = []
        for index, occurrence in enumerate(occurrences):
            for c in sorted(occurrence.prerequisites, key=lambda c: tuple(sorted(c))):
                producer = achieved[c]
                if producer is not None:
                    edges.append((producer, index, c))
            for c in occurrence.supported:
                achieved[c] = index
        return Evaluation(len(occurrences), tuple(o.operator for o in occurrences),
                          occurrences, tuple(edges), self.version)

    def __call__(self, node):
        return self.evaluate(node).value

    def calc_h_with_plan(self, node):
        result = self.evaluate(node)
        return result.value, None if result.plan is None else {op.name for op in result.plan}

    def get_call_counter(self):
        return self.call_counter

    def refine(self, node):
        """Add one suffix-regression conflict conjunction.

        This conservative refinement policy is not Keyder's BSG ranking.
        Regress the sequential plan from the goal until a delete conflict is
        found, then add that simultaneous requirement. Return False for a real
        plan, a recognized dead end, or when this policy finds no new candidate.
        """
        result = self.evaluate(node)
        if result.plan is None:
            return False
        required = frozenset(self.task.goals)
        for op in reversed(result.plan):
            if (op.del_effects - op.add_effects) & required:
                if required not in self.graph.conjunctions:
                    return self.add_conjunction(required)
            required = (required - op.add_effects) | op.preconditions
        if not required <= node.state and required not in self.graph.conjunctions:
            return self.add_conjunction(required)
        return False

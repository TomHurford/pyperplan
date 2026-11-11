from itertools import combinations

import pytest

from pyperplan.heuristics.conjunctions import hCFFHeuristic
from pyperplan.planner import HEURISTICS, SEARCHES
from pyperplan.search.searchspace import make_root_node
from pyperplan.task import Operator, Task


def make_task(initial, goals, operators):
    facts = set(initial) | set(goals)
    for op in operators:
        facts.update(op.preconditions | op.add_effects | op.del_effects)
    return Task("hcff", facts, frozenset(initial), frozenset(goals), operators)


def validate_relaxed(heuristic, state, result):
    achieved = set(heuristic.graph.contained_in(state))
    for occurrence in result.occurrences:
        assert occurrence.prerequisites <= achieved
        regression = occurrence.operator.preconditions
        for c in occurrence.supported:
            assert occurrence.operator.add_effects & c
            assert not (occurrence.operator.del_effects - occurrence.operator.add_effects) & c
            regression |= c - occurrence.operator.add_effects
        assert occurrence.prerequisites == heuristic.graph.contained_in(regression)
        achieved.update(occurrence.supported)
    assert heuristic.graph.contained_in(heuristic.task.goals) <= achieved
    assert len(result.plan) == result.value


def test_discovery_and_empty_preconditions():
    task = make_task([], ["p", "q"], [Operator("both", [], ["p", "q"], [])])
    heuristic = HEURISTICS["hcff"](task)
    result = heuristic.evaluate(make_root_node(task.initial_state))
    assert result.value == 1
    validate_relaxed(heuristic, task.initial_state, result)
    assert heuristic(make_root_node(frozenset({"p", "q"}))) == 0
    assert heuristic(make_root_node(task.initial_state)) == 1


def test_refinement_recognizes_impossible_joint_goal():
    task = make_task(["p"], ["p", "q"], [Operator("consume", ["p"], ["q"], ["p"])])
    heuristic = hCFFHeuristic(task)
    node = make_root_node(task.initial_state)
    assert heuristic(node) == 1
    assert heuristic.refine(node)
    assert heuristic.version == 1
    assert heuristic(node) == float("inf")
    assert not heuristic.refine(node)
    assert SEARCHES["rhc"](task, heuristic) is None


def test_cross_context_requires_repeated_action_occurrences():
    ops = [Operator("p", [], ["p"], []),
           Operator("q1", [], ["q1"], ["q2", "p"]),
           Operator("q2", [], ["q2"], ["q1", "p"])]
    task = make_task([], ["p", "q1", "q2"], ops)
    heuristic = hCFFHeuristic(task, [{"p", "q1"}, {"p", "q2"}, {"q1", "q2"}])
    assert heuristic(make_root_node(task.initial_state)) == float("inf")
    # q1/q2 is reachable, but more expensive than either individual context.
    ops.append(Operator("joint", ["q1"], ["q2"], ["p"]))
    task = make_task([], ["p", "q1", "q2"], ops)
    heuristic = hCFFHeuristic(task, [{"p", "q1"}, {"p", "q2"}, {"q1", "q2"}])
    result = heuristic.evaluate(make_root_node(task.initial_state))
    validate_relaxed(heuristic, task.initial_state, result)


def test_full_conjunctions_extract_real_plans_for_every_state():
    ops = [Operator("out", ["a"], ["b"], ["a"]),
           Operator("back", ["b"], ["a"], ["b"]),
           Operator("deliver", ["b"], ["g"], [])]
    task = make_task(["a"], ["a", "g"], ops)
    facts = sorted(task.facts)
    conjunctions = [frozenset(c) for n in range(1, len(facts)+1) for c in combinations(facts, n)]
    heuristic = hCFFHeuristic(task, conjunctions)
    for n in range(len(facts)+1):
        for subset in combinations(facts, n):
            state = frozenset(subset)
            result = heuristic.evaluate(make_root_node(state))
            if result.plan is None:
                assert not state & {"a", "b"}
                continue
            validate_relaxed(heuristic, state, result)
            for op in result.plan:
                assert op.applicable(state)
                state = op.apply(state)
            assert task.goal_reached(state)


@pytest.mark.parametrize("search", ["ehc", "eehc", "gehc", "hb_ehc", "hybrid_ehc", "gbf", "rhc"])
def test_search_integration(search):
    task = make_task(["a"], ["a", "g"], [
        Operator("out", ["a"], ["b"], ["a"]),
        Operator("back", ["b"], ["a"], ["b"]),
        Operator("deliver", ["b"], ["g"], [])])
    heuristic = hCFFHeuristic(task, [{"a", "g"}])
    plan = SEARCHES[search](task, heuristic)
    assert plan is not None
    state = task.initial_state
    for op in plan:
        assert op.applicable(state)
        state = op.apply(state)
    assert task.goal_reached(state)


def test_random_small_tasks_against_explicit_relaxation_and_real_search():
    import random
    from pyperplan.search import breadth_first_search
    rng = random.Random(17)
    facts = ["a", "b", "c"]
    subsets = [frozenset(c) for n in range(4) for c in combinations(facts, n)]
    for _ in range(60):
        ops = []
        for i in range(5):
            pre, add, delete = (rng.choice(subsets) for _ in range(3))
            ops.append(Operator(str(i), pre, add, delete - add))
        task = Task("random", set(facts), rng.choice(subsets), rng.choice(subsets), ops)
        conjunctions = [c for c in subsets if c and rng.choice([False, True])]
        h = hCFFHeuristic(task, conjunctions)
        result = h.evaluate(make_root_node(task.initial_state))
        if result.plan is None:
            assert breadth_first_search(task) is None
        else:
            validate_relaxed(h, task.initial_state, result)
        full = hCFFHeuristic(task, subsets[1:])
        result = full.evaluate(make_root_node(task.initial_state))
        actual = breadth_first_search(task)
        assert (result.plan is None) == (actual is None)
        if result.plan is not None:
            state = task.initial_state
            for op in result.plan:
                assert op.applicable(state)
                state = op.apply(state)
            assert task.goal_reached(state)


def test_repeated_operator_is_not_deduplicated():
    task = make_task(["a"], ["g1", "g2"], [
        Operator("out", ["a"], ["b"], ["a"]),
        Operator("one", ["b"], ["a", "g1"], ["b"]),
        Operator("two", ["b"], ["a", "g2"], ["b"])])
    facts = sorted(task.facts)
    conjunctions = [c for n in range(1, len(facts)+1) for c in combinations(facts, n)]
    h = hCFFHeuristic(task, conjunctions)
    result = h.evaluate(make_root_node(task.initial_state))
    assert [op.name for op in result.plan].count("out") == 2
    validate_relaxed(h, task.initial_state, result)


def test_refinement_search_fallback_and_timeout():
    task = make_task(["a"], ["g"], [Operator("go", ["a"], ["g"], [])])
    h = hCFFHeuristic(task)
    assert SEARCHES["rhc"](task, h, max_time=0) is None
    assert SEARCHES["rhc"](task, h, max_refinements=0) is not None
    with pytest.raises(ValueError):
        SEARCHES["rhc"](task, h, horizon=0)

import pytest

from pyperplan.heuristics.conjunctions import ConjunctionGraph
from pyperplan.task import Operator, Task


def task(operators=()):
    return Task("conjunctions", {"p", "q", "r"}, frozenset(), {"p", "q"}, operators)


def test_singletons_are_always_present_and_invalid_conjunctions_rejected():
    graph = ConjunctionGraph(task(), [{"p", "q"}])
    assert graph.conjunctions == {frozenset(c) for c in [("p",), ("q",), ("r",), ("p", "q")]}
    assert graph.contained_in({"p", "q"}) == {frozenset(c) for c in [("p",), ("q",), ("p", "q")]}
    with pytest.raises(ValueError):
        ConjunctionGraph(task(), [{"missing"}])
    with pytest.raises(ValueError):
        ConjunctionGraph(task(), [set()])


def test_regression_requires_preserved_facts_and_conjunctive_preconditions():
    op = Operator("achieve", {"r"}, {"p"}, set())
    graph = ConjunctionGraph(task([op]), [{"p", "q"}, {"q", "r"}])
    counter = graph.achievers[frozenset({"p", "q"})][0]
    assert counter.regression == {"q", "r"}
    assert counter.prerequisites == {frozenset({"q"}), frozenset({"r"}), frozenset({"q", "r"})}
    assert counter in graph.dependents[frozenset({"q", "r"})]


def test_deleting_a_conjunct_prevents_achievement():
    op = Operator("conflict", set(), {"p"}, {"q"})
    graph = ConjunctionGraph(task([op]), [{"p", "q"}])
    assert not graph.achievers[frozenset({"p", "q"})]
    assert len(graph.achievers[frozenset({"p"})]) == 1


def test_no_precondition_counter_and_rebuilding_after_refinement():
    op = Operator("free", set(), {"p"}, set())
    original = ConjunctionGraph(task([op]))
    assert not original.achievers[frozenset({"p"})][0].prerequisites
    refined = ConjunctionGraph(task([op]), [{"p", "q"}])
    assert refined.achievers[frozenset({"p", "q"})][0].regression == {"q"}


def test_reachability_detects_delete_conflict_and_resets_between_states():
    op = Operator("achieve", {"q"}, {"p"}, {"q"})
    graph = ConjunctionGraph(task([op]), [{"p", "q"}])
    assert frozenset({"p"}) in graph.reachable({"q"})
    assert frozenset({"p", "q"}) not in graph.reachable({"q"})
    assert frozenset({"p", "q"}) in graph.reachable({"p", "q"})
    assert frozenset({"p"}) not in graph.reachable(set())


def test_reachability_fires_empty_precondition_counters():
    op = Operator("free", set(), {"p"}, set())
    graph = ConjunctionGraph(task([op]))
    assert frozenset({"p"}) in graph.reachable(set())

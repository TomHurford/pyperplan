"""Behavioral regressions for the custom searches and planner integration.

Heuristic call budgets turn runaway lookaheads into deterministic failures,
without threads, sleeps, or waiting for the production four-minute timeout.
"""

import logging

import pytest

from pyperplan.planner import SEARCHES, find_domain
from pyperplan.search.benchmarking import Benchmark
from pyperplan.task import Operator, Task


CUSTOM_SEARCHES = ["ehc", "eehc", "gehc", "hb_ehc", "hybrid_ehc"]


class BoundedHeuristic:
    name = "regression"

    def __init__(self, values, limit=40):
        self.values = values
        self.limit = limit
        self.calls = 0

    def __call__(self, node):
        self.calls += 1
        assert self.calls <= self.limit, (
            "Search exceeded heuristic call budget on a tiny finite graph; "
            "check cycle detection and lookahead timeout handling"
        )
        return self.values[next(iter(node.state))]


def graph_task(edges, goal="goal", initial="start"):
    facts = {initial, goal}
    operators = []
    for name, source, target in edges:
        facts.update([source, target])
        operators.append(Operator(name, {source}, {target}, {source}))
    return Task("regression", facts, frozenset({initial}), {goal}, operators)


@pytest.fixture
def benchmark_logging(monkeypatch):
    # Isolate algorithm behavior from the separate CLI-only logging defect.
    monkeypatch.setattr(logging.Logger, "benchmarks", lambda *args: None, raising=False)


def assert_valid_plan(task, solution):
    assert solution is not None
    state = task.initial_state
    for operator in solution:
        assert operator in task.operators
        assert operator.applicable(state)
        state = operator.apply(state)
    assert task.goal_reached(state)


@pytest.mark.parametrize("search", CUSTOM_SEARCHES)
def test_custom_search_accepts_initial_goal(search, benchmark_logging):
    task = graph_task([], goal="start")
    assert SEARCHES[search](task, BoundedHeuristic({"start": 0})) == []


@pytest.mark.parametrize("search", CUSTOM_SEARCHES)
def test_custom_search_returns_executable_plan(search, benchmark_logging):
    task = graph_task([("step", "start", "middle"), ("finish", "middle", "goal")])
    heuristic = BoundedHeuristic({"start": 2, "middle": 1, "goal": 0})
    solution = SEARCHES[search](task, heuristic)
    assert_valid_plan(task, solution)
    assert len(solution) == 2


@pytest.mark.parametrize("search", CUSTOM_SEARCHES)
def test_custom_search_exhausts_acyclic_dead_end(search, benchmark_logging):
    task = graph_task([("step", "start", "dead")])
    assert SEARCHES[search](task, BoundedHeuristic({"start": 1, "dead": 1})) is None


@pytest.mark.parametrize("search", CUSTOM_SEARCHES)
def test_custom_search_exhausts_finite_cycle(search, benchmark_logging):
    task = graph_task([("out", "start", "other"), ("back", "other", "start")])
    heuristic = BoundedHeuristic({"start": 1, "other": 1})
    assert SEARCHES[search](task, heuristic) is None


@pytest.mark.parametrize("search", CUSTOM_SEARCHES)
def test_custom_search_checks_timeout_inside_lookahead(
    search, benchmark_logging, monkeypatch
):
    # A finite chain avoids conflating timeout enforcement with cycle detection.
    task = graph_task([(str(i), str(i), str(i + 1)) for i in range(100)], initial="0")
    heuristic = BoundedHeuristic({str(i): 1 for i in range(101)})
    monkeypatch.setattr(Benchmark, "time_up", lambda self: heuristic.calls >= 3)
    assert SEARCHES[search](task, heuristic) is None
    assert heuristic.calls <= 5, "Expired lookahead continued evaluating states"


@pytest.mark.parametrize("search", CUSTOM_SEARCHES)
def test_custom_search_checks_timeout_while_generating_successors(
    search, benchmark_logging, monkeypatch
):
    task = graph_task([(str(i), "start", str(i)) for i in range(100)])
    values = {str(i): 1 for i in range(100)}
    values["start"] = 1
    heuristic = BoundedHeuristic(values)
    monkeypatch.setattr(Benchmark, "time_up", lambda self: heuristic.calls >= 3)
    assert SEARCHES[search](task, heuristic) is None
    assert heuristic.calls <= 5, "Expired expansion continued evaluating successors"


def test_hb_ehc_backtracks_after_improving_dead_end(benchmark_logging):
    task = graph_task([
        ("a-trap", "start", "dead"),
        ("b-detour", "start", "detour"),
        ("finish", "detour", "goal"),
    ])
    heuristic = BoundedHeuristic({"start": 2, "dead": 1, "detour": 3, "goal": 0})
    assert_valid_plan(task, SEARCHES["hb_ehc"](task, heuristic))


def test_benchmark_logging_works_without_cli_initialization(monkeypatch):
    monkeypatch.delattr(logging.Logger, "benchmarks", raising=False)
    benchmark = Benchmark("task", "heuristic", "search", "lookahead", "ordering")
    benchmark.start_timer()
    benchmark.log_solution([], "Already solved")
    assert benchmark.solution_found
    assert benchmark.solution_length == 0


@pytest.mark.parametrize("search", CUSTOM_SEARCHES)
def test_custom_search_works_without_cli_initialization(search, monkeypatch):
    monkeypatch.delattr(logging.Logger, "benchmarks", raising=False)
    task = graph_task([("finish", "start", "goal")])
    heuristic = BoundedHeuristic({"start": 1, "goal": 0})
    assert_valid_plan(task, SEARCHES[search](task, heuristic))


def test_hb_ehc_backtracks_past_multiple_failed_improvements(benchmark_logging):
    task = graph_task([
        ("a-trap", "start", "trap"),
        ("deeper", "trap", "dead"),
        ("b-detour", "start", "detour"),
        ("finish", "detour", "goal"),
    ])
    heuristic = BoundedHeuristic(
        {"start": 3, "trap": 2, "dead": 1, "detour": 4, "goal": 0}
    )
    assert_valid_plan(task, SEARCHES["hb_ehc"](task, heuristic))


@pytest.mark.parametrize("problem", ["problem.pddl", "task01.pddl"])
def test_find_domain_uses_default_domain(tmp_path, problem):
    domain = tmp_path / "domain.pddl"
    domain.write_text("; domain inference only checks the path\n")
    assert find_domain(str(tmp_path / problem)) == str(domain)


def test_find_domain_selects_matching_numbered_domain(tmp_path):
    (tmp_path / "domain.pddl").touch()
    numbered = tmp_path / "domain02.pddl"
    numbered.touch()
    assert find_domain(str(tmp_path / "task02.pddl")) == str(numbered)

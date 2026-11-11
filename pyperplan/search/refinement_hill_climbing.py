"""Bounded lookahead with explicit heuristic refinement and GBFS fallback."""

from collections import deque
import time

from . import searchspace
from .a_star import greedy_best_first_search


def refinement_hill_climbing(task, heuristic, use_preferred_ops=False,
                             horizon=3, max_refinements=100, max_time=240):
    if not hasattr(heuristic, "refine"):
        raise ValueError("Refinement search requires a refinable heuristic (hcff)")
    if horizon < 1 or max_refinements < 0:
        raise ValueError("horizon must be positive and max_refinements nonnegative")
    deadline = time.monotonic() + max_time
    root = searchspace.make_root_node(task.initial_state)
    current = root
    refinements = 0

    def evaluate(node):
        if time.monotonic() >= deadline:
            raise TimeoutError
        return heuristic(node)

    try:
        while True:
            if task.goal_reached(current.state):
                return current.extract_solution()
            base = evaluate(current)
            if base == float("inf"):
                if current is root:
                    return None
                current = root
                continue
            queue = deque([current])
            visited = {current.state}
            better = None
            while queue and better is None:
                node = queue.popleft()
                if node.g - current.g >= horizon:
                    continue
                for operator, state in task.get_successor_states(node.state):
                    if state in visited:
                        continue
                    visited.add(state)
                    child = searchspace.make_child_node(node, operator, state)
                    if task.goal_reached(state):
                        return child.extract_solution()
                    value = evaluate(child)
                    if value < base:
                        better = child
                        break
                    if value != float("inf"):
                        queue.append(child)
            if better is not None:
                current = better
            elif refinements < max_refinements and heuristic.refine(current):
                refinements += 1
                # All lookahead values belong to the old C and are discarded.
            else:
                # A capped or exhausted refinement policy is not a dead-end proof.
                # Restart complete graph search with the final, fixed C.
                return greedy_best_first_search(task, evaluate)
    except TimeoutError:
        return None

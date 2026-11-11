"""
This implements two variations of the Heaps and Backtracking EHC search algorithm
"""

import heapq
import logging

from . import searchspace
from .benchmarking import Benchmark

# This is the original implementation from the paper
def hb_enforced_hill_climbing(planning_task, heuristic, use_preferred_ops=False):
    initial_node = searchspace.make_root_node(planning_task.initial_state)

    c = 100
    l = 10
    queue = []
    closed = set()
    current_node = initial_node

    if planning_task.goal_reached(current_node.state):
        return current_node.extract_solution()
    best_value = heuristic(current_node)
    closed.add(current_node)
    heapq.heappush(queue, (best_value, current_node))
    while queue:
        _, current_node = heapq.heappop(queue)

        successors = planning_task.get_successor_states(current_node.state)

        for operator, successor in successors:
            successor_node = searchspace.make_child_node(current_node, operator, successor)

            if successor_node in closed:
                continue
            closed.add(successor_node)

            if planning_task.goal_reached(successor_node.state):
                return successor_node.extract_solution()
            
            successor_h_val = heuristic(successor_node)

            if successor_h_val < best_value:
                best_value = successor_h_val
                best_local_state = successor_node
                queue = []
                heapq.heappush(queue, (successor_h_val, successor_node))
                break

            heapq.heappush(queue, successor_node)
            if len(queue) > c:
                return None
        if (len(queue) == 0) and (l > 0):
            if best_local_state == initial_node:
                return None
            else:
                l -= 1
                heapq.heappush(queue, best_local_state.get_parent())
    return None

def hbehc(planning_task, heuristic, use_preferred_ops=False):
    """Use bounded greedy lookahead, backtracking after failed improvements."""
    logger = Benchmark(planning_task.name, heuristic.name, "hb_ehc", "GBFS", "None")
    logger.start_timer()
    max_queue_size = 10000
    backtracks_left = 50
    failed_states = set()

    def gbfs_lookahead(start_node):
        expansion_count = 0
        heuristic_count = 1
        base_value = heuristic(start_node)
        queue = [(base_value, 0, start_node)]
        visited_states = {start_node.state}

        def finish(node, message):
            logger.log_lookahead(
                node is not None, expansion_count, heuristic_count, 0, message
            )
            return node

        while queue:
            if logger.time_up():
                return finish(None, "Timeout")
            _, _, node = heapq.heappop(queue)
            for operator, successor in planning_task.get_successor_states(node.state):
                if logger.time_up():
                    return finish(None, "Timeout")
                if successor in failed_states or successor in visited_states:
                    continue
                visited_states.add(successor)
                successor_node = searchspace.make_child_node(node, operator, successor)
                expansion_count += 1
                if planning_task.goal_reached(successor):
                    return finish(successor_node, "Goal found in lookahead")
                value = heuristic(successor_node)
                heuristic_count += 1
                if value == float("inf"):
                    continue
                if value < base_value:
                    return finish(successor_node, "Better state found in lookahead")
                heapq.heappush(queue, (value, expansion_count, successor_node))
                if len(queue) > max_queue_size:
                    return finish(None, "Queue length limit reached")
        return finish(None, "Lookahead exhausted")

    current_node = searchspace.make_root_node(planning_task.initial_state)
    while current_node is not None:
        if planning_task.goal_reached(current_node.state):
            solution = current_node.extract_solution()
            logger.log_solution(solution, "Solution found")
            return solution
        if logger.time_up():
            logger.log_solution(None, "Timeout")
            return None

        next_node = gbfs_lookahead(current_node)
        if logger.time_up():
            logger.log_solution(None, "Timeout")
            return None
        if next_node is not None:
            current_node = next_node
            continue

        # Exclude the failed improvement before revisiting its parent, so the
        # next lookahead can explore alternatives instead of choosing it again.
        failed_states.add(current_node.state)
        if current_node.parent is None or backtracks_left == 0:
            break
        backtracks_left -= 1
        current_node = current_node.parent
        logger.restart()

    logger.log_solution(None, "No solution found")
    return None

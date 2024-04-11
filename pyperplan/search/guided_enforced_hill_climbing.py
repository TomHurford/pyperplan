"""
This is an implementation of the Guided Enforced Hill Climbing Algorithm
using the least failed first operator heuristic.
"""

from collections import deque
import logging

from . import searchspace
from .benchmarking import Benchmark
from ..ordering import LeastFailedFirst

def guided_enforced_hill_climbing(planning_task, heuristic, use_preferred_ops=False):
    logger = Benchmark(planning_task.name, heuristic.name, "guided_ehc", "BFS", "LFF")
    logging.info("Starting Guided Enforced Hill Climbing Search")

    def bfs(start_node):
        best_h_val = heuristic(start_node)
        queue = deque([start_node])
        start_depth = start_node.g
        current_depth = start_depth -1
        visited_states = set()

        expansion_count = 0
        heuristic_calls = 1
        ordering_calls = 0

        while queue:
            node = queue.popleft()

            node_h_value = heuristic(node)
            heuristic_calls += 1

            if node.state in visited_states:
                logging.debug("PRUNED: Node visited")
                continue
            visited_states.add(node.state)

            for operator, successor in ordering.successor_generator(node.state):
                if successor in visited_states:
                    logging.debug("PRUNED: Successor visited")
                    continue

                expansion_count += 1

                successor_node = searchspace.make_child_node(node, operator, successor)

                successor_h_value = heuristic(successor_node)
                heuristic_calls += 1

                if planning_task.goal_reached(successor_node.state):
                    logging.debug("Goal found in lookahead")
                    logger.log_lookahead(True, expansion_count, heuristic_calls, ordering_calls, "Goal found")

                if successor_h_value == float('inf'):
                    continue
                elif successor_h_value < best_h_val:
                    logging.info(f"Better heuristic state found in lookahead")
                    logging.debug(f"LOOKAHEAD SUCCESS")
                    logging.debug(f"EXPANSIONS: {expansion_count}")
                    logging.debug(f"LOOKAHEAD DEPTH: {successor_node.g - start_depth}")
                    logging.debug(f"HEURISTIC CALLS: {heuristic_calls}")
                    logging.debug(f"ORDERING CALLS: {ordering_calls}")
                    logger.log_lookahead(True, expansion_count, heuristic_calls, ordering_calls, "Successor found")
                    return successor_node
                
                ordering.update_action_failure_weight(node_h_value, operator, successor_h_value)
                ordering_calls += 1
                queue.append(successor_node)

            if node.g > current_depth:
                current_depth = node.g
                logging.debug(f"Lookahead depth: {current_depth - start_depth}")

        logger.log_lookahead(False, expansion_count, heuristic_calls, ordering_calls, "Lookahead exhausted")
        return None
    
    logger.start_timer()

    ordering = LeastFailedFirst(planning_task)
    initial_node = searchspace.make_root_node(planning_task.initial_state)
    current_node = initial_node

    while current_node is not None:
        if planning_task.goal_reached(current_node.state):
            solution = current_node.extract_solution()
            logging.info("Solution found")
            logger.log_solution(solution, "Solution found")
            return solution
        if logger.time_up():
            logging.info(f"Timeout after {logger.max_time}")
            logger.log_solution(None, f"Timeout after {logger.max_time}")
            return None
        
        current_node = bfs(current_node)

    logging.info("No solution found")
    logger.log_solution(None, "No solution found")
    return None
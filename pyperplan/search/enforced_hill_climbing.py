"""
Implements the enforced hill climbing search algorithm.
"""

from collections import deque
import logging

from . import searchspace
from .benchmarking import Benchmark

def enforced_hill_climbing(planning_task, heuristic, use_preferred_ops=False):
    # Logging
    logger = Benchmark(planning_task.name, heuristic.name, "classic_ehc", "BFS", "None")
    logging.info("Starting Enforced Hill Climbing Search")

    def bfs(start_node):
        best_h_val = heuristic(start_node)
        queue = deque([start_node])
        start_depth = start_node.g
        visited_states = set()

        expansion_count = 0
        heuristic_calls = 1

        while queue:
            if logger.time_up():
                logging.debug("Timeout")
                logger.log_lookahead(False, expansion_count, heuristic_calls, 0, "Timeout")
                return None
            node = queue.popleft()

            if node.state in visited_states:
                logging.debug("PRUNED: Node visited")
                continue
            visited_states.add(node.state)

            successors = planning_task.get_successor_states(node.state)

            for operator, successor in successors:
                if successor in visited_states:
                    logging.debug("PRUNED: Successor visited")
                    continue

                expansion_count += 1

                successor_node = searchspace.make_child_node(node, operator, successor)

                successor_h_value = heuristic(successor_node)
                heuristic_calls += 1

                if successor_h_value == float('inf'):
                    continue
                elif successor_h_value < best_h_val:
                    logging.info(f"Better heuristic state found in lookahead")
                    logging.debug(f"LOOKAHEAD SUCCESS")
                    logging.debug(f"EXPANSIONS: {expansion_count}")
                    logging.debug(f"LOOKAHEAD DEPTH: {successor_node.g - start_depth}")
                    logging.debug(f"HEURISTIC CALLS: {heuristic_calls}")
                    logger.log_lookahead(True, expansion_count, heuristic_calls, 0, "Successor found")
                    return successor_node

                queue.append(successor_node)            

        # If our queue is empty then we have exhausted the lookahead search 
        # space, and should return None
        logger.log_lookahead(False, expansion_count, heuristic_calls, 0, "Lookahead exhausted")
        return None

    logger.start_timer()

    initial_node = searchspace.make_root_node(planning_task.initial_state)
    current_node = initial_node

    while current_node is not None:
        current_node = bfs(current_node)
        
        if logger.time_up():
            logging.info(f"Timeout after {logger.max_time}")
            logger.log_solution(None, "Time limit reached")
            return None
        
        if planning_task.goal_reached(current_node.state):
            solution = current_node.extract_solution()
            logging.info("Solution found")
            logger.log_solution(solution, "Solution found")
            return solution
        


    logging.info("No solution found")
    logger.log_solution(None, "No solution found")
    return None
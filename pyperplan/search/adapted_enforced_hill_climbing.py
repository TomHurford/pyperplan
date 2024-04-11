"""
Implements the Heaped Variant Enforced Hill Climbing Search Algorithm
"""

import heapq
import logging

from . import searchspace
from .benchmarking import Benchmark

def adapted_enforced_hill_climbing(planning_task, heuristic, use_preferred_ops=False):
    logger = Benchmark(planning_task.name, heuristic.name, "adapted_ehc", "BeFS", "None")
    logging.info("Starting Adapted Enforced Hill Climbing Search")

    def best_first_search(start_node):
        base_h_val = heuristic(start_node)        
        pqueue = []
        visited_states = set()
        start_depth = start_node.g

        expansion_count = 0
        
        heapq.heappush(pqueue, (base_h_val, expansion_count, start_node))

        heuristic_calls = 1

        while pqueue:
            _, _, node = heapq.heappop(pqueue)

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

                if planning_task.goal_reached(successor_node.state):
                    logging.debug("Goal found in lookahead")
                    logger.log_lookahead(True, expansion_count, heuristic_calls, 0, "Goal found")
                    return successor_node
                
                successor_h_value = heuristic(successor_node)
                heuristic_calls += 1

                if successor_h_value == float('inf'):
                    continue
                elif successor_h_value < base_h_val:
                    logging.info(f"Better heuristic state found in lookahead")
                    logging.debug(f"LOOKAHEAD SUCCESS")
                    logging.debug(f"EXPANSIONS: {expansion_count}")
                    logging.debug(f"LOOKAHEAD DEPTH: {successor_node.g - start_depth}")
                    logging.debug(f"HEURISTIC CALLS: {heuristic_calls}")
                    logger.log_lookahead(True, expansion_count, heuristic_calls, 0, "Successor found")
                    return successor_node

                heapq.heappush(pqueue, (successor_h_value, expansion_count, successor_node))

                if len(pqueue) > max_queue_size:
                    logging.debug("Queue size exceeded")
                    logger.log_lookahead(False, expansion_count, heuristic_calls, 0, "Queue size exceeded")
                    return None

        logger.log_lookahead(False, expansion_count, heuristic_calls, 0, "Lookahead exhausted")
        return None
    
    logger.start_timer()

    initial_node = searchspace.make_root_node(planning_task.initial_state)
    current_node = initial_node
    max_queue_size = 10000

    while current_node is not None:
        if planning_task.goal_reached(current_node.state):
            solution = current_node.extract_solution()
            logging.info("Solution Found")
            logger.log_solution(solution, "Solution Found")
            return solution
        if logger.time_up():
            logging.info(f"Timeout after {logger.max_time}")
            logger.log_solution(None, f"Timeout after {logger.max_time}")
            return None
        
        current_node = best_first_search(current_node)

    logging.info("No Solution Found")
    logger.log_solution(None, "No Solution Found")
    return None
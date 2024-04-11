"""
This is my proposed optimised EHC algorithm.
"""

import heapq
import logging

from pyperplan.ordering.least_failed_first import LeastFailedFirst

from . import searchspace
from .benchmarking import Benchmark

def combined_enforced_hill_climbing(planning_task, heuristic, use_preferred_ops = False):
    logger = Benchmark(planning_task.name, heuristic.name, "super_ehc", "BeFS", "LFF")
    logging.info("Starting Enforced Hill Climbing Search+")

    def best_first_search(start_node):
        heuristic_calls = 0
        expansion_count = 0
        ordering_calls = 0
        # If the state of the start node is in the cache, return the cached value if it is not calculate the value and store it in the cache
        if start_node.state in state_heuristic_cache:
            base_h_val = state_heuristic_cache[start_node.state]
        else:
            base_h_val = heuristic(start_node)
            heuristic_calls += 1
            state_heuristic_cache[start_node.state] = base_h_val
        pqueue = []
        visited_states = set()

        heapq.heappush(pqueue, (base_h_val, expansion_count, start_node))

        while pqueue:
            if logger.time_up():
                logging.info("Timeout")
                return None
            node_h_val, _, node = heapq.heappop(pqueue)

            if node.state in dead_end_cache:
                logging.debug("PRUNED: Node in dead-end cache")
                continue

            if node.state in visited_states:
                logging.debug("PRUNED: Node visited")
                continue
            visited_states.add(node.state)

            for operator, successor in ordering.successors_generator(node.state):
                if successor in dead_end_cache:
                    logging.debug("PRUNED: Successor in dead-end cache")
                    continue
                if successor in visited_states:
                    logging.debug("PRUNED: Successor visited")
                    continue

                successor_node = searchspace.make_child_node(node, operator, successor)
                expansion_count += 1

                if planning_task.goal_reached(successor_node.state):
                    logging.debug("Goal found in lookahead")
                    logger.log_lookahead(True, expansion_count, heuristic_calls, ordering_calls, "Goal found")
                    return successor_node
                
                if successor in state_heuristic_cache:
                    successor_h_value = state_heuristic_cache[successor]
                else:
                    successor_h_value = heuristic(successor_node)
                    heuristic_calls += 1
                    state_heuristic_cache[successor] = successor_h_value

                if successor_h_value == float('inf'):
                    continue
                elif successor_h_value < base_h_val:
                    logging.info(f"Better heuristic state found in lookahead")
                    logging.debug(f"LOOKAHEAD SUCCESS")
                    logging.debug(f"EXPANSIONS: {expansion_count}")
                    logging.debug(f"LOOKAHEAD DEPTH: {successor_node.g - start_node.g}")
                    logging.debug(f"HEURISTIC CALLS: {heuristic_calls}")
                    logging.debug(f"ORDERING CALLS: {ordering_calls}")
                    logger.log_lookahead(True, expansion_count, heuristic_calls, ordering_calls, "Successor found")
                    return successor_node
                
                ordering.update_action_failure_weight(node_h_val, operator, successor_h_value)
                ordering_calls += 1

                heapq.heappush(pqueue, (successor_h_value, expansion_count, successor_node))

                if len(pqueue) >= max_queue_size:
                    logging.debug("Queue size exceeded")
                    logger.log_lookahead(False, expansion_count, heuristic_calls, ordering_calls, "Queue size exceeded")
                    return None

        logger.log_lookahead(False, expansion_count, heuristic_calls, ordering_calls, "Lookahead exhausted")
        return None
    
    logger.start_timer()

    max_queue_size = 10000
    dead_end_cache = set()
    ordering = LeastFailedFirst(planning_task)
    state_heuristic_cache = {}
    restart_count = 0

    initial_node = searchspace.make_root_node(planning_task.initial_state)
    current_node = initial_node

    while initial_node.state not in dead_end_cache:
        next_node = best_first_search(current_node)

        if logger.time_up():
            logging.info("Time limit reached")
            logger.log_solution(None, "Time limit reached")
            return None
        
        if next_node is None:
            dead_end_cache.add(current_node.state)
            logging.info("Dead-end found search restarted")
            logger.restart()
            restart_count += 1
            logging.debug(f"Restart count: {restart_count}")
            logging.debug(f"Dead-end cache size: {len(dead_end_cache)}")
            current_node = initial_node
            continue
        
        current_node = next_node
        
        if planning_task.goal_reached(current_node.state):
            solution = current_node.extract_solution()
            logging.info("Solution found")
            logger.log_solution(solution, "Solution found")
            return solution
        
    logging.info("No solution found")
    logger.log_solution(None, "No solution found")
    return None

            
            





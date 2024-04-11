"""
Implements the Heaps and Backtracking Enforced Hill Climbing Search Algorithm
"""

import heapq
import logging

from . import searchspace
from .benchmarking import Benchmark

def db_adapted_enforced_hill_climbing(planning_task, heuristic, use_preferred_ops=False):
    logger = Benchmark(planning_task.name, heuristic.name, "db_adapted_ehc", "DB_BeFS", "None")
    logging.info("Starting Depth Bound Adapted Enforced Hill Climbing Search")

    def depth_bound_best_first_search(start_node):
        base_h_val = heuristic(start_node)
        start_depth = start_node.g
        visited_states = set()
        pqueue = []

        expansion_count = 0
        heapq.heappush(pqueue, (base_h_val, expansion_count, start_node))
        
        heuristic_count = 1

        while pqueue:
            if logger.time_up():
                logging.info("Timeout")
                return None
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
                
                successor_h_value = heuristic(successor_node)
                heuristic_count += 1    

                if successor_h_value == float('inf'):
                    continue
                elif successor_h_value < base_h_val:
                    logging.info(f"Better heuristic state found in lookahead")
                    logging.debug(f"LOOKAHEAD SUCCESS")
                    logging.debug(f"EXPANSIONS: {expansion_count}")
                    logging.debug(f"LOOKAHEAD DEPTH: {successor_node.g - start_depth}")
                    logger.log_lookahead(True, expansion_count, heuristic_count, 0, "Successor found")
                    return successor_node
                
                if successor_node.g - start_depth >= depth_bound:
                    logging.debug("Successor not added to queue, beyond depth bound")
                    continue

                heapq.heappush(pqueue, (successor_h_value, expansion_count, successor_node))

                if len(pqueue) > max_queue_size:
                    logging.debug("Queue size exceeded")
                    logger.log_lookahead(False, expansion_count, heuristic_count, 0, "Queue size exceeded")
                    return None
                
        logger.log_lookahead(False, expansion_count, heuristic_count, 0, "Lookahead exhausted")
        return None
    
    logger.start_timer()

    initial_node = searchspace.make_root_node(planning_task.initial_state)
    depth_bound = 5
    current_node = initial_node
    max_queue_size = 10000
    dead_end_cache = set()
    restart_count = 0

    while initial_node.state not in dead_end_cache:
        next_node = depth_bound_best_first_search(current_node)
        
        if logger.time_up():
            logging.info(f"Time limit reached")
            logger.log_solution(None, "Time limit reached")
            return None
        
        if next_node is None:
            dead_end_cache.add(current_node.state)
            logging.info("Dead end found search restarted")
            logger.restart()
            restart_count += 1
            logging.debug(f"Restart count: {restart_count}")
            current_node = initial_node
            continue
        
        current_node = next_node
        
        if planning_task.goal_reached(current_node.state):
            solution = current_node.extract_solution()
            logging.info("Solution Found")
            logger.log_solution(solution, "Solution Found")
            return solution
        
    logging.info("No solution Found")
    logger.log_solution(None, "No solution Found")
    return None
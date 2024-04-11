"""
Implements the enforced hill climbing search algorithm.
"""

from collections import deque
import logging
import time

from . import searchspace
from .benchmarking import Benchmark

def episodic_enforced_hill_climbing(planning_task, heuristic, use_preferred_ops=False):
    logger = Benchmark(planning_task.name, heuristic.name, "episodic_ehc", "DB_BFS", "None")
    logging.info("Starting Episodic Enforced Hill Climbing Search")

    def db_bfs(start_node):
        best_h_val = heuristic(start_node)
        queue = deque([start_node])
        start_depth = start_node.g
        current_depth = start_depth - 1
        visited_states = set()

        expansion_count = 0
        heuristic_calls = 1

        while queue:
            node = queue.popleft()

            if node.state in dead_end_cache:
                logging.debug("PRUNED: Node in dead-end cache")
                continue

            if node.state in visited_states:
                logging.debug("PRUNED: Node visited")                
                continue
            visited_states.add(node.state)

            successors = planning_task.get_successor_states(node.state)

            for operator, successor in successors:
                if successor in dead_end_cache:
                    logging.debug("PRUNED: Successor in dead-end cache")
                    continue
                if successor in visited_states:
                    logging.debug("PRUNED: Successor visited")
                    continue


                successor_node = searchspace.make_child_node(node, operator, successor)
                expansion_count += 1

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
                
                if successor_node.g - start_depth >= depth_bound:
                    logging.debug("Successor not added to queue, beyond depth bound")
                    continue

                queue.append(successor_node)

            if node.g > current_depth:
                current_depth = node.g
                logging.debug(f"Lookahead depth: {current_depth - start_depth}")

        logger.log_lookahead(False, expansion_count, heuristic_calls, 0, "Lookahead exhausted")
        return None
                
    logger.start_timer()

    dead_end_cache = set()
    initial_node = searchspace.make_root_node(planning_task.initial_state)
    current_node = initial_node
    depth_bound = 7
    restart_count = 0

    while initial_node.state not in dead_end_cache:
        if logger.time_up():
            logging.info("Time limit reached")
            logger.log_solution(None, "Time limit reached")
            return None
        
        next_node = db_bfs(current_node)
        if next_node is None:
            dead_end_cache.add(current_node.state)
            logging.info("Dead end found, search restarted")
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
            
    # If the initial node is added to the dead-end cache, then the problem is 
    # not solvable with the current lookahead function/depth
    logging.info("No Solution Found")
    logger.log_solution(None, "No solution found")
    return None
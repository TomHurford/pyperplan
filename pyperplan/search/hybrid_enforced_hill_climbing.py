"""
This is my proposed optimised EHC algorithm.
"""

import heapq
import logging

from pyperplan.ordering.least_failed_first import LeastFailedFirst

from . import searchspace
from .benchmarking import Benchmark

def hybrid_enforced_hill_climbing(planning_task, heuristic, use_preferred_ops = False):
    # Initialize the benchmark logger
    logger = Benchmark(planning_task.name, heuristic.name, "hybrid_ehc", "BeFS", "LFF")
    logging.info("Starting Enforced Hill Climbing Search+")

    # Define the Bounded Best-First Search (BBFS) function
    def bounded_best_first_search(start_node):
        # Initialize counters and variables
        heuristic_calls = 0
        expansion_count = 0
        ordering_calls = 0
        # Check if the start node's state is in the heuristic cache
        if start_node.state in state_heuristic_cache:
            base_h_val = state_heuristic_cache[start_node.state]
        else:
            # If not, compute the heuristic value and store it in the cache
            base_h_val = heuristic(start_node)
            heuristic_calls += 1
            state_heuristic_cache[start_node.state] = base_h_val
        pqueue = []
        visited_states = set()

        # Add the start node to the priority queue
        heapq.heappush(pqueue, (base_h_val, expansion_count, start_node))

        # Main loop for BBFS
        while pqueue:
            node_h_val, _, node = heapq.heappop(pqueue)

            # Skip if node is in the dead-end cache
            if node.state in dead_end_cache:
                logging.debug("PRUNED: Node in dead-end cache")
                continue

            # Skip if node is already visited
            if node.state in visited_states:
                logging.debug("PRUNED: Node visited")
                continue
            visited_states.add(node.state)

            # Generate successors
            for operator, successor in ordering.successor_generator(node.state):
                # Skip if successor is in the dead-end cache or already visited
                if successor in dead_end_cache:
                    logging.debug("PRUNED: Successor in dead-end cache")
                    continue
                if successor in visited_states:
                    logging.debug("PRUNED: Successor visited")
                    continue

                # Create a new node for the successor
                successor_node = searchspace.make_child_node(node, operator, successor)
                expansion_count += 1

                # Check if goal is reached
                if planning_task.goal_reached(successor_node.state):
                    logging.debug("Goal found in lookahead")
                    logger.log_lookahead(True, expansion_count, heuristic_calls, ordering_calls, "Goal found")
                    return successor_node
                
                # Check if the successor's state is in the heuristic cache
                if successor in state_heuristic_cache:
                    successor_h_value = state_heuristic_cache[successor]
                else:
                    # If not, compute the heuristic value and store it in the cache
                    successor_h_value = heuristic(successor_node)
                    heuristic_calls += 1
                    state_heuristic_cache[successor] = successor_h_value

                # Skip if the heuristic value is infinity
                if successor_h_value == float('inf'):
                    continue
                # Check if the successor is better than the base node
                elif successor_h_value < base_h_val:
                    logging.info(f"Better heuristic state found in lookahead")
                    logging.debug(f"LOOKAHEAD SUCCESS")
                    logging.debug(f"EXPANSIONS: {expansion_count}")
                    logging.debug(f"LOOKAHEAD DEPTH: {successor_node.g - start_node.g}")
                    logging.debug(f"HEURISTIC CALLS: {heuristic_calls}")
                    logging.debug(f"ORDERING CALLS: {ordering_calls}")
                    logger.log_lookahead(True, expansion_count, heuristic_calls, ordering_calls, "Successor found")
                    return successor_node
                
                # Update the failure weight of the action
                ordering.update_action_failure_weight(node_h_val, operator, successor_h_value)
                ordering_calls += 1

                # Add the successor to the priority queue
                heapq.heappush(pqueue, (successor_h_value, expansion_count, successor_node))

                # Check if the queue size limit is reached
                if len(pqueue) >= max_queue_size:
                    logging.debug("Queue size exceeded")
                    logger.log_lookahead(False, expansion_count, heuristic_calls, ordering_calls, "Queue size exceeded")
                    return None

        # If the queue is exhausted, return None
        logger.log_lookahead(False, expansion_count, heuristic_calls, ordering_calls, "Lookahead exhausted")
        return None
    
    # Start the timer for the benchmark
    logger.start_timer()

    # Initialize parameters
    max_queue_size = 10000
    dead_end_cache = set()
    ordering = LeastFailedFirst(planning_task)
    state_heuristic_cache = {}
    restart_count = 0

    # Create the initial node
    initial_node = searchspace.make_root_node(planning_task.initial_state)
    current_node = initial_node

    # Main loop for the Hybrid Enforced Hill Climbing (HEHC) algorithm
    while initial_node.state not in dead_end_cache:
        # Perform BBFS
        next_node = bounded_best_first_search(current_node)

        # Check if time is up
        if logger.time_up():
            logging.info("Time limit reached")
            logger.log_solution(None, "Time limit reached")
            return None
        
        # Process the result of BBFS
        if next_node is None:
            # If BBFS failed, add the current state to the dead-end cache and restart the search
            dead_end_cache.add(current_node.state)
            logging.info("Dead-end found search restarted")
            logger.restart()
            restart_count += 1
            logging.debug(f"Restart count: {restart_count}")
            logging.debug(f"Dead-end cache size: {len(dead_end_cache)}")
            current_node = initial_node
            continue
        
        # Update the current node
        current_node = next_node
        
        # Check if goal is reached
        if planning_task.goal_reached(current_node.state):
            solution = current_node.extract_solution()
            logging.info("Solution found")
            logger.log_solution(solution, "Solution found")
            return solution
        
    # If the search exhausts, return None
    logging.info("No solution found")
    logger.log_solution(None, "No solution found")
    return None
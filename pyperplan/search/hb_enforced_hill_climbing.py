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

# This is a conversion of the structure to match the lookahead function and handler structure introduced in my dissertation
def hbehc(planning_task, heuristic, use_preferred_ops=False):
    # Initialize the benchmark logger
    logger = Benchmark(planning_task.name, heuristic.name, "hb_ehc", "GBFS", "None")
    logging.info("Starting Heaps and Backtracking Enforced Hill Climbing Search")

    # Define the Greedy Best-First Search (GBFS) lookahead function
    def gbfs_lookahead(start_node):
        # Initialize counters and variables
        expansion_count = 0
        heuristic_count = 0
        nonlocal best_value, best_local_state
        queue = []
        closed = set()
        heapq.heappush(queue, (heuristic(start_node), expansion_count, start_node))
        heuristic_count += 1

        # Main loop for GBFS lookahead
        while queue:
            _, _, node = heapq.heappop(queue)

            # Skip if node is already closed
            if node in closed:
                continue
            closed.add(node)
            expansion_count += 1

            # Check if goal is reached
            if planning_task.goal_reached(node.state):
                logging.info("Goal found in lookahead")
                logger.log_lookahead(True, expansion_count, heuristic_count, 0, "Goal found in lookahead")
                return node
            
            # Generate successors
            successors = planning_task.get_successor_states(node.state)
            for operator, successor in successors:
                successor_node = searchspace.make_child_node(node, operator, successor)
                expansion_count += 1
                successor_h_val = heuristic(successor_node)
                heuristic_count += 1

                # Check if successor is better than the best known state
                if successor_h_val < best_value:
                    best_value = successor_h_val
                    best_local_state = successor_node
                    logging.debug("Better state found in lookahead")
                    logger.log_lookahead(True, expansion_count, heuristic_count, 0, "Better state found in lookahead")
                    return successor_node
                
                # Add successor to the queue
                heapq.heappush(queue, (successor_h_val, expansion_count, successor_node))

            # Check if queue length limit is reached
            if len(queue) > c:
                logging.debug("Queue length limit reached")
                logger.log_lookahead(False, expansion_count, heuristic_count, 0, "Queue length limit reached")
                return None
            
        # If queue is exhausted, return None
        logging.debug("Lookahead exhausted")
        logger.log_lookahead(False, expansion_count, heuristic_count, 0, "Lookahead exhausted")
        return None
    
    # Start the timer for the benchmark
    logger.start_timer()
    # Create the initial node
    initial_node = searchspace.make_root_node(planning_task.initial_state)

    # Initialize parameters
    c = 10000
    l = 50
    queue = []
    closed = set()
    current_node = initial_node
    best_local_state = None

    # Check if the initial state is the goal
    if planning_task.goal_reached(current_node.state):
        solution = current_node.extract_solution()
        logging.info("Solution found at start")
        logger.log_solution(solution, "Solution found at start")
        return current_node.extract_solution()
    
    # Compute the heuristic value of the initial state
    best_value = heuristic(current_node)
    closed.add(current_node)
    heapq.heappush(queue, (best_value, -1, current_node))

    # Main loop for the Enforced Hill Climbing (EHC) algorithm
    while queue:
        # Check if time is up
        if logger.time_up():
            logging.info("Timeout")
            logger.log_solution(None, "Timeout")
            return None
                
        # Perform GBFS lookahead
        next_node = gbfs_lookahead(current_node)

        # Process the result of the lookahead
        if next_node:
            # If goal is reached, return the solution
            if planning_task.goal_reached(next_node.state):
                solution = next_node.extract_solution()
                logging.info("Solution found")
                logger.log_solution(solution, "Solution found")
                return solution
            # Update the current node
            current_node = next_node
            continue
        elif len(queue) == 0 and l > 0:
            # If queue is empty and backtracks are left, perform a backtrack
            l -= 1
            if best_local_state != initial_node:
                current_node = best_local_state.get_parent()
                logging.info("Backtracking to best local state")
                continue
            logging.info("No backtracks left, exiting search")
            logger.log_solution(None, "No backtracks left")
            return None # Exhausts with no backtracking left
        logging.info("Lookahead failed to find successor")
        logger.log_solution(None, "Lookahead failed")
        return None # Lookahead failed, no backtrack
    logging.info("Queue exhausted")
    logger.log_solution(None, "Queue exhausted")
    return None # Just in case
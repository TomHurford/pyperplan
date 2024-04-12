"""
Implements the Episodic Enforced Hill Climbing search algorithm.
"""

from collections import deque
import logging

from . import searchspace
from .benchmarking import Benchmark

def episodic_enforced_hill_climbing(planning_task, heuristic, use_preferred_ops=False):
    # Initialize the benchmark logger
    logger = Benchmark(planning_task.name, heuristic.name, "episodic_ehc", "BFS", "None")

    # Define the breadth-first search (BFS) function
    def bfs(start_node):
        # Initialize the best heuristic value and the open list
        best_h_val = heuristic(start_node)
        open_list = deque([start_node])

        # Initialize counters for expansions and heuristic calls
        expansion_count = 0
        heuristic_calls = 1
        
        # Start the BFS loop
        while open_list:
            # Check if time is up and log the result
            if logger.time_up():
                logging.info("Timeout")
                return None
            
            # Get the next node from the open list
            node = open_list.popleft()

            # If the node is in the dead-end cache, skip it
            if node.state in dead_end_cache:
                logging.debug("PRUNED: Node in dead-end cache")
                continue

            # Get the successors of the current node
            successors = planning_task.get_successor_states(node.state)

            # Loop through the successors
            for operator, successor in successors:
                # If the successor is in the dead-end cache, skip it
                if successor in dead_end_cache:
                    logging.debug("PRUNED: Successor in dead-end cache")
                    continue

                # Create a new node for the successor
                successor_node = searchspace.make_child_node(node, operator, successor)
                expansion_count += 1

                # Calculate the heuristic value of the successor node
                successor_h_value = heuristic(successor_node)
                heuristic_calls += 1

                # If the heuristic value is infinity, skip the successor
                if successor_h_value == float('inf'):
                    continue
                # If the heuristic value is better than the best so far, return the successor node
                elif successor_h_value < best_h_val:
                    logging.debug("Better successor found")
                    logger.log_lookahead(True, expansion_count, heuristic_calls, 0, "Successor found")
                    return successor_node
                
                # Add the successor node to the open list
                open_list.append(successor_node)
        
        # If the open list is empty, log the result and return None
        logger.log_lookahead(False, expansion_count, heuristic_calls, 0, "Dead end found")
        return None
    
    # Start the timer
    logger.start_timer()

    # Initialize the dead-end cache and the initial node
    dead_end_cache = set()
    initial_node = searchspace.make_root_node(planning_task.initial_state)
    current_node = initial_node
    restart_count = 0

    # Start the main loop
    while initial_node.state not in dead_end_cache:
        # Perform BFS from the current node
        next_node = bfs(current_node)

        # If time is up, log the result and return None
        if logger.time_up():
            logging.info("Time limit reached")
            logger.log_solution(None, "Time limit reached")
            return None
        
        # If no next node was found, add the current node to the dead-end cache and restart the search
        if next_node is None:
            dead_end_cache.add(current_node.state)
            logging.info("Dead end found, restarting search")
            logger.restart()
            restart_count += 1
            logging.debug(f"Restart count: {restart_count}")
            current_node = initial_node
            continue
        current_node = next_node

        # If the goal is reached, extract the solution, log it, and return it
        if planning_task.goal_reached(current_node.state):
            solution = current_node.extract_solution()
            logging.info("Solution found")
            logger.log_solution(solution, "Solution found")
            return solution
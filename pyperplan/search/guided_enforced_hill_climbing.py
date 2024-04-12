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
    # Initialize the benchmark logger
    logger = Benchmark(planning_task.name, heuristic.name, "guided_ehc", "BFS", "LFF")
    logging.info("Starting Guided Enforced Hill Climbing Search")

    # Define the breadth-first search (BFS) function
    def bfs(start_node):
        # Initialize the best heuristic value and the open list
        best_h_val = heuristic(start_node)
        queue = deque([start_node])

        # Initialize counters for expansions, heuristic calls, and ordering calls
        expansion_count = 0
        heuristic_calls = 1
        ordering_calls = 0

        # Start the BFS loop
        while queue:
            # Check if time is up and log the result
            if logger.time_up():
                logging.info("Timeout")
                return None
            
            # Get the next node from the open list
            node = queue.popleft()

            # Calculate the heuristic value of the current node
            node_h_value = heuristic(node)
            heuristic_calls += 1

            # Loop through the successors of the current node
            for operator, successor in ordering.successor_generator(node.state):
                # Increment the expansion count
                expansion_count += 1

                # Create a new node for the successor
                successor_node = searchspace.make_child_node(node, operator, successor)

                # Calculate the heuristic value of the successor node
                successor_h_value = heuristic(successor_node)
                heuristic_calls += 1

                # If the heuristic value is infinity, skip the successor
                if successor_h_value == float('inf'):
                    continue
                # If the heuristic value is better than the best so far, log the result and return the successor node
                elif successor_h_value < best_h_val:
                    logging.info(f"Better heuristic state found in lookahead")
                    logger.log_lookahead(True, expansion_count, heuristic_calls, ordering_calls, "Successor found")
                    return successor_node
                
                # Update the failure weight of the action and increment the ordering calls count
                ordering.update_action_failure_weight(node_h_value, operator, successor_h_value)
                ordering_calls += 1

                # Add the successor node to the open list
                queue.append(successor_node)

        # If the open list is empty, log the result and return None
        logger.log_lookahead(False, expansion_count, heuristic_calls, ordering_calls, "Lookahead exhausted")
        return None
    
    # Start the timer
    logger.start_timer()

    # Initialize the least failed first ordering and the initial node
    ordering = LeastFailedFirst(planning_task)
    initial_node = searchspace.make_root_node(planning_task.initial_state)
    current_node = initial_node

    # Start the main loop
    while current_node is not None:
        # If the goal is reached, extract the solution, log it, and return it
        if planning_task.goal_reached(current_node.state):
            solution = current_node.extract_solution()
            logging.info("Solution found")
            logger.log_solution(solution, "Solution found")
            return solution
        # If time is up, log the result and return None
        if logger.time_up():
            logging.info(f"Timeout after {logger.max_time}")
            logger.log_solution(None, "Time limit reached}")
            return None
        
        # Perform BFS from the current node
        current_node = bfs(current_node)

    # If no solution was found, log the result and return None
    logging.info("No solution found")
    logger.log_solution(None, "No solution found")
    return None
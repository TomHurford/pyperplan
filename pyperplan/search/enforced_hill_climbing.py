"""
Implements the EHC search algorithm.
"""

from collections import deque
import logging

from . import searchspace
from .benchmarking import Benchmark

def enforced_hill_climbing(planning_task, heuristic, use_preferred_ops=False):
    # Initialize logging
    logger = Benchmark(planning_task.name, heuristic.name, "classic_ehc", "BFS", "None")
    logging.info("Starting Enforced Hill Climbing Search")

    # Define the breadth-first search (BFS) function
    def bfs(start_node):
        # Initialize the best heuristic value, queue, depth, and visited states
        best_h_val = heuristic(start_node)
        queue = deque([start_node])

        # Initialize counters for expansions and heuristic calls
        expansion_count = 0
        heuristic_calls = 1

        # Start the BFS loop
        while queue:
            node = queue.popleft()

            # Get the successors of the current node
            successors = planning_task.get_successor_states(node.state)

            # Loop through the successors
            for operator, successor in successors:
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
                # If the heuristic value is better than the best so far, return the successor node
                elif successor_h_value < best_h_val:
                    logging.info(f"Better heuristic state found in lookahead")
                    logger.log_lookahead(True, expansion_count, heuristic_calls, 0, "Successor found")
                    return successor_node

                # Add the successor node to the queue
                queue.append(successor_node)            

        # If the queue is empty, the lookahead search space is exhausted
        logger.log_lookahead(False, expansion_count, heuristic_calls, 0, "Lookahead exhausted")
        return None

    # Start the timer
    logger.start_timer()

    # Initialize the root node and the current node
    initial_node = searchspace.make_root_node(planning_task.initial_state)
    current_node = initial_node

    # Start the main loop
    while current_node is not None:
        # Perform BFS from the current node
        current_node = bfs(current_node)
        # Check if time is up and log the result
        if logger.time_up():
            logging.debug("Timeout")
            logger.log_solution(None, "Timeout")
            return None

        if current_node is None:
            logging.info("No successor returned from lookahead")
            logger.log_solution(None, "No successor returned from lookahead")
            return None
        
        # If time is up, log the result and return None
        if logger.time_up():
            logging.info(f"Timeout after {logger.max_time}")
            logger.log_solution(None, "Time limit reached")
            return None
        
        # If the goal is reached, extract the solution, log it, and return it
        if planning_task.goal_reached(current_node.state):
            solution = current_node.extract_solution()
            logging.info("Solution found")
            logger.log_solution(solution, "Solution found")
            return solution
        

    # If the main loop ends without finding a solution, log the result and return None
    logging.info("No solution found")
    logger.log_solution(None, "No solution found")
    return None
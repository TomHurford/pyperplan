"""
Implements a depth bounded lookahead variation of the enforced hill climbing search algorithm.
"""

from collections import deque
import logging

from . import searchspace
from .benchmarking import Benchmark

def depth_bound_enforced_hill_climbing(planning_task, heuristic, use_preferred_ops=False):
    logger = Benchmark(planning_task.name, heuristic.name, "depthbound_ehc")

    def db_bfs(start_node):
        best_h_val = heuristic(start_node)
        logger.heuristic_call
        queue = deque([start_node])
        start_depth = start_node.g
        current_depth = start_depth - 1
        visited_states = set()

        expansion_count = 0
        expansions_at_depth = 0

        while queue:
            # Get the node from the queue
            node = queue.popleft()

            # Check that the state has not been visited in this lookahead
            if node.state in visited_states:
                continue
            visited_states.add(node.state)

            successors = planning_task.get_successor_states(node.state)
            logger.successor_state_expansion(len(successors))

            for operator, successor in successors:
                # Check that the successor state, should not be pruned
                if successor in visited_states:
                    continue

                # Increment expansion counts
                expansion_count += 1
                expansions_at_depth += 1

                # Make the successor into a search node
                successor_node = searchspace.make_child_node(node, operator, successor)
                logger.search_node_creation()

                # Check if successor is goal_state
                if planning_task.goal_reached(successor_node.state):
                    return successor_node

                # Calculate heuristic value
                successor_h_value = heuristic(successor_node)
                logger.heuristic_call()


                if successor_h_value == float('inf'):
                    continue
                # If the successors heuristic value is better than the initial 
                # nodes heuristic then return the successor node 
                elif successor_h_value < best_h_val:
                    logging.debug(f"Successor state found in lookahead"
                        f" - Lookahead depth: {successor_node.g - start_depth}"
                        f" - Expansions at depth: {expansions_at_depth}"
                        f" - Total lookahead expansions: {expansion_count}")
                    logging.info(f"Successor found in lookahead"
                        f" - Lookahead depth: {successor_node.g - start_depth}"
                        f" - Total expansions: {expansion_count}")
                    logger.expansion_since_improvement(expansion_count)
                    return successor_node
                
                # If the successor node is outside the depth of the search do 
                # not add it to the queue
                if successor_node.g - start_depth >= depth_bound:
                    continue

                # Add the successor to the back of the queue
                queue.append(successor_node)

            # Purely for printing
            if node.g > current_depth:
                current_depth = node.g
                logging.debug(f"Lookahead depth: {current_depth - start_depth}"
                              f" - Expansions at depth: {expansions_at_depth}")
                expansions_at_depth = 0

        # If our queue is empty then we have exhausted the lookahead search 
        # space, and should return None
        return None
                
    # Set the depth bound
    depth_bound = 5

    initial_node = searchspace.make_root_node(planning_task.initial_state)
    logger.search_node_creation()

    current_node = initial_node

    while current_node is not None:
        if planning_task.goal_reached(current_node.state):
            solution = current_node.extract_solution()
            logger.log_solution(solution, "Solution found")

        current_node = db_bfs(current_node)
            
    logging.info("No Solution Found")
    logger.log_solution(None, "No solution found")
    return None
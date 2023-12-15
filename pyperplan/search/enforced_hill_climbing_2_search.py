import logging
from ..utils import priority_queue
from . import searchspace
from . import breadth_first_search


def enforced_hillclimbing_search2(planning_task, heuristic, use_preferred_ops=False):
    """
    This function implements a simplified version of the Enforced Hill Climbing (EHC) algorithm.

    Parameters:
    planning_task (object): The planning task to be solved.
    heuristic (function): The heuristic function to be used.
    use_preferred_ops (bool): Whether to use preferred operators or not. Default is False.

    Returns:
    object: The solution node if a solution is found, otherwise None.

    """

    logging.info("Starting Enforced Hill Climbing Search")

    # Initialize a priority queue
    pqueue = priority_queue.HeuristicPriorityQueue()

    # Create the root node from the initial state of the planning task
    initial_node = searchspace.make_root_node(planning_task.initial_state)
    # Calculate the heuristic value of the initial node
    best_value = heuristic(initial_node)
    # Add the initial node to the priority queue
    pqueue.add_state(initial_node, best_value)

    logging.debug("Initial node added to the priority queue")

    # Initialize a set to keep track of visited states
    visited = set()

    # Main loop of the algorithm
    while not pqueue.empty():
        # Pop a node from the priority queue
        node = pqueue.pop()
        # Add the state of the node to the visited set
        visited.add(node.state)

        logging.debug("Popped a node from the priority queue")

        # Check if the goal has been reached
        if planning_task.goal_reached(node.state):
            # If the goal has been reached, extract and return the solution
            logging.info("Goal reached, extracting solution")
            return node.extract_solution()

        # Generate successor states
        for operator, successor_state in planning_task.get_successor_states(node.state):
            # Check if the successor state has been visited before
            if successor_state not in visited:
                # Create a new node from the successor state
                new_node = searchspace.make_child_node(node, operator, successor_state)
                # Calculate the heuristic value of the new node
                new_value = heuristic(new_node)

                logging.debug("Generated a new successor state")

                # Check if the new value is better than the best value found so far
                if new_value < best_value:
                    # If it is, update the best value, reset the priority queue and add the new node to it
                    best_value = new_value
                    pqueue.reset()
                    pqueue.add_state(new_node, new_value)
                    logging.debug("Found a better state, resetting the priority queue")
                    break
                else:
                    # If it's not, simply add the new node to the priority queue
                    pqueue.add_state(new_node, new_value)

    # If the algorithm has gone through all states without finding a solution, log the information and start a BFS
    logging.info("No Goal found using Enforced Hill Climbing")
    logging.info("Starting BFS")
    bfs_solution = breadth_first_search(planning_task)

    # If a solution is found using BFS, return it
    if bfs_solution is not None:
        logging.info("Goal found using BFS")
        return bfs_solution
    else:
        # If no solution is found using BFS, log the information and return None
        logging.info("No Goal found using BFS")
        return None

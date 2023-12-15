from . import searchspace
from icecream import ic
import logging


def episodic_ehc_search(planning_task, heuristic):
    logging.info("Starting Episodic Enforced Hill Climbing Search")
    logging.info("-----------------------------------------------")

    """
    Algorithm:
        1. Cross episodic dead end set (cache) to hold states that are dead ends
        2. Initialise the first state and create a variable to hold the best state
        3. While the initial state has not been added to the dead end set
            3.1 Run Breadth First Search to find the best successor state (the one with the lowest heuristic value) 
                that is not a dead end
            3.2 If there is no successor state, add the current state to the dead end set and reset the current state to
                the initial state
            3.3 Else, set the current state to the best successor state
            3.4 Check if the current state is a goal state, if it is, return the solution
        4. If no solution return None
    """

    dead_ends = set()
    initial_node = searchspace.make_root_node(planning_task.initial_state)
    current_node = initial_node

    while current_node.state not in dead_ends:
        best_successor_node = None
        best_heuristic_value = float('inf')

        for operator, successor_state in planning_task.get_successor_states(current_node.state):
            if successor_state not in dead_ends:
                successor_node = searchspace.make_child_node(current_node, operator, successor_state)
                heuristic_value = heuristic(successor_node)
                if heuristic_value < best_heuristic_value:
                    best_heuristic_value = heuristic_value
                    best_successor_node = successor_node

        if best_successor_node is None:
            dead_ends.add(current_node.state)
            current_node = initial_node
        else:
            current_node = best_successor_node

        if planning_task.goal_reached(current_node.state):
            logging.info("Goal reached. Start extraction of solution.")
            return current_node.extract_solution()

    logging.info("No solution found")
    return None

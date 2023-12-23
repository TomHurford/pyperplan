from . import searchspace
from icecream import ic
import logging


def episodic_ehc_search(planning_task, heuristic):
    logging.info("Starting Episodic Enforced Hill Climbing Search")
    logging.info("-----------------------------------------------")

    dead_ends = set()
    initial_node = searchspace.make_root_node(planning_task.initial_state)
    current_node = initial_node
    visited = set()
    old_count = 0

    logging.debug("Initial state: {}".format(initial_node))

    while initial_node.state not in dead_ends:
        if current_node.state not in visited:
            visited.add(current_node.state)
            logging.debug("New State")
        else:
            logging.debug("Already Visited State")
            old_count += 1
        best_successor_node = None
        best_heuristic_value = heuristic(current_node)

        count = 0
        for operator, successor_state in planning_task.get_successor_states(current_node.state):
            count += 1
            if successor_state not in dead_ends:
                successor_node = searchspace.make_child_node(current_node, operator, successor_state)
                heuristic_value = heuristic(successor_node)
                if heuristic_value < best_heuristic_value:
                    best_heuristic_value = heuristic_value
                    best_successor_node = successor_node
                    break

        logging.debug("REPEATED VISITS: {}".format(old_count))
        logging.debug("DEAD-ENDS: {}".format(len(dead_ends)))
        logging.debug("STATES EXPANDED: {}".format(len(visited)))
        logging.debug("# SUCCESSORS: {}".format(count))
        logging.debug("CURRENT H VALUE: {}".format(heuristic(current_node)))
        logging.debug("IMPROVED H VALUE: {}".format(best_heuristic_value))

        if best_successor_node is None:
            dead_ends.add(current_node.state)
            current_node = initial_node
            logging.debug("DEAD-END, BACKTRACKING")
        else:
            current_node = best_successor_node
            logging.debug("CONTINUE SEARCH")

        if planning_task.goal_reached(current_node.state):
            logging.info("Goal reached. Start extraction of solution.")
            logging.debug("Number of dead ends: {}".format(len(dead_ends)))
            logging.debug("Number of visited states: {}".format(len(visited)))
            logging.debug("Number of repeated visits: {}".format(old_count))

            return current_node.extract_solution()

    logging.info("No solution found")
    return None

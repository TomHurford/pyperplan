from . import searchspace
from ..utils import priority_queue
import logging


def adapted_enforced_hill_climbing_search(planning_task, heuristic):
    logging.info("Starting Adapted Enforced Hill Climbing Search")

    max_queue_size = 10000000
    max_backtracks = 100

    queue = priority_queue.HeuristicPriorityQueue()
    closed = set()

    initial_state = searchspace.make_root_node(planning_task.initial_state)
    best_state = initial_state
    best_h = heuristic(initial_state)

    closed.add(initial_state)
    queue.add_state(initial_state, best_h)

    logging.info("Initial state added to the queue")

    while not queue.empty():
        current_state = queue.pop()

        logging.debug("Popped a state from the queue")

        for operator, successor_state in planning_task.get_successor_states(current_state.state):
            if successor_state not in closed:
                closed.add(successor_state)
                successor_node = searchspace.make_child_node(current_state, operator, successor_state)
                if planning_task.goal_reached(successor_state):
                    logging.info("Goal reached, extracting solution")
                    return successor_node.extract_solution()
                else:
                    h = heuristic(successor_node)
                    if h < best_h:
                        best_h = h
                        best_state = successor_node
                        queue.reset()
                        queue.add_state(successor_node, h)
                        logging.debug("Found a better state, resetting the queue")
                        break
                    else:
                        queue.add_state(successor_node, h)
                        if len(queue) > max_queue_size:
                            logging.debug("Queue full")
                            return None
        if queue.empty and max_backtracks > 0:
            if best_state == initial_state:
                logging.debug("Backtracked to start")
                return None
            else:
                logging.debug("Backtracking")
                max_backtracks -= 1
                queue.add_state(best_state, best_h)
    logging.info("No solution found")
    return None
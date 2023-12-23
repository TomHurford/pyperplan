from ..utils import priority_queue
from . import searchspace
import logging
from icecream import ic
import time


def adapted_enforced_hill_climbing_search(planning_task, heuristic):

    logging.info("Starting Adapted Enforced Hill Climbing Search")

    start_time = time.time()
    max_queue_size = 10000000
    max_backtracks = 100

    queue = priority_queue.PriorityQueue()
    closed = set()
    initial_state = searchspace.make_root_node(planning_task.initial_state)
    best_state = initial_state
    best_h = heuristic(initial_state)
    closed.add(initial_state)
    queue.add_item(initial_state, best_h)

    logging.info("Initial state added to the queue")
    logging.debug("Initial state: {}".format(initial_state))

    while not queue.empty():
        current_state = queue.pop()
        for operator, successor_state in planning_task.get_successor_states(current_state.state):
            if successor_state not in closed:
                closed.add(successor_state)
                successor_node = searchspace.make_child_node(current_state, operator, successor_state)
                if planning_task.goal_reached(successor_state):
                    logging.info("Goal reached, extracting solution")
                    return successor_node.extract_solution()
                else:
                    h = heuristic(successor_node)
                    logging.debug("Heuristic value of successor node: {}".format(h))
                    if h < best_h:
                        best_h = h
                        best_state = successor_node
                        queue.reset()
                        queue.add_item(successor_node, h)
                        logging.debug("Found a better state, resetting the queue")
                        break
                    else:
                        queue.add_item(successor_node, h)
                        logging.debug("Added successor node to the queue")
                        if len(queue) > max_queue_size:
                            logging.info("Queue full")
                            return None
        if start_time + 300 < time.time():
            logging.info("Time limit exceeded")
            return None
        if queue.empty() and max_backtracks > 0:
            if best_state == initial_state:
                logging.info("Backtracked to start")
                return None
            else:
                logging.info("Backtracking")
                max_backtracks -= 1
                queue.add_item(current_state, heuristic(current_state))
    logging.info("No solution found")
    return None

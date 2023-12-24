from . import searchspace
from ..ordering import least_failed_first
from ..utils import queue
from icecream import ic

import logging


def guided_enforced_hill_climbing_search(planning_task, heuristic):
    """
    GHC Algorithm:

    Inputs:  <state s, actions a, goals G>
    1. h_s <- h(s)
    2. Open_list <- successors_of(s)
    3. while Open_list is not empty do
    4.     next_state <- pop_best_of(Open_list, sigma)
    5.     h_next_state <- h(next_state)
    6.     if h_next_state = 0 then
    7.         return success
    8.     else if h_next_state < h_s then
    9.         return GHC(next_state, a, G)
    10.    else
    11.        Open_list <- Open_list + successors_of(next_state)
    12.    update_ordering_measure(s, next_state, sigma)
    13. return "A dead end has occurred"


    @param planning_task:
    @param heuristic:
    @return:
    """
    ordering_heuristic = least_failed_first.LeastFailedFirst(heuristic, planning_task)
    initial_node = searchspace.make_root_node(planning_task.initial_state)

    def ghc(node):
        heuristic_value = heuristic(node)
        open_list = planning_task.get_successor_states(node.state)
        while len(open_list) > 0:
            operator, successor = ordering_heuristic.pop_best_of(open_list)
            open_list.remove((operator, successor))
            successor_node = searchspace.make_child_node(node, operator, successor)
            if planning_task.goal_reached(successor):
                logging.info("Goal reached")
                return successor_node.extract_solution()
            elif heuristic(successor_node) < heuristic_value:
                logging.info("Reached successor node")
                return ghc(successor_node)
            else:
                open_list += planning_task.get_successor_states(successor_node.state)
            logging.info("Updating ordering measure")
            ordering_heuristic.update_ordering_measure(successor_node)
        logging.info("A dead end has occurred")
        return None

    return ghc(initial_node)

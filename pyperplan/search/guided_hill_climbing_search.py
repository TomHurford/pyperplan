from . import searchspace
from ..ordering import least_failed_first
from ..utils import queue

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
    ordering_function = least_failed_first.LeastFailedFirst(heuristic, planning_task)
    open_list = queue.Queue()

    initial_state = searchspace.make_root_node(planning_task.initial_state)
    heuristic_value = heuristic(initial_state)

    successors = planning_task.get_successor_states(initial_state.state)

    for successor in successors:
        open_list.push(successor)

    def ghc(state):
        while not open_list.is_empty():
            best_operator = ordering_function(open_list.list(), state.state)
            logging.debug("Best operator: {}".format(best_operator.name))
            if best_operator is None:
                logging.info("Action based dead end")
                return None
            next_state = searchspace.make_child_node(state, best_operator, best_operator.apply(state))
            if planning_task.goal_reached(next_state.state):
                logging.info("Goal reached, extracting solution")
                return next_state.extract_solution()
            elif heuristic(next_state) < heuristic_value:
                logging.info("Better state found")
                ghc(next_state)
            else:
                next_state_successors = planning_task.get_successor_states(next_state.state)
                for next_state_successor in next_state_successors:
                    open_list.push(next_state_successor)
            ordering_function.update(next_state)
        logging.info("No solution found, dead end")
        return None

    plan = ghc(initial_state)

    if plan is not None:
        return plan
    return None

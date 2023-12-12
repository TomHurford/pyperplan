from . import searchspace
from ..utils import priority_queue
import logging
from icecream import ic


def adapted_enforced_hill_climbing(planning_task, heuristic):
    max_queue_size = 10000000
    max_backtracks = 100

    queue = priority_queue.HeuristicPriorityQueue()
    closed = set()

    initial_state = searchspace.make_root_node(planning_task.initial_state)
    best_state = initial_state
    best_h = heuristic(initial_state)

    closed.add(initial_state)
    queue.add_state(initial_state, best_h)

    while not queue.empty():
        current_state = queue.pop()

        for operator, successor_state in planning_task.get_successor_states(current_state.state):
            if successor_state not in closed:
                closed.add(successor_state)
                successor_node = searchspace.make_child_node(current_state, operator, successor_state)
                if planning_task.goal_reached(successor_state):
                    return successor_node.extract_solution()
                else:
                    h = heuristic(successor_node)
                    if h < best_h:
                        best_h = h
                        best_state = successor_node
                        queue.reset()
                        queue.add_state(successor_node, h)
                        break
                    else:
                        queue.add_state(successor_node, h)
                        if len(queue) > max_queue_size:
                            print("Queue full")
                            return None
        if not queue.empty and max_backtracks > 0:
            if best_state == initial_state:
                print("Backtracked to start")
                return None
            else:
                print("Backtracking")
                max_backtracks -= 1
                queue.add_state(best_state, best_h)
    print("No solution found")
    return None

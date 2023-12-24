from .ordering_base import Ordering
import heapq


class LeastFailedFirst(Ordering):
    """
    This class implements the Least Failed First ordering heuristic.
    """

    def __init__(self, heuristic, task):
        super().__init__(heuristic)
        self.action_failure_weights = {}

        for operator in task.operators:
            self.action_failure_weights[operator] = 0

    def pop_best_of(self, open_list):
        """
        Open list contains pairs of (operator, successor_state), where the operator is the action applied to reach the
        successor state. The object contains a dictionary of action failure weights, which is used to determine the
        order of the open list. The open list is ordered by the action failure weights of the operators.

        @param open_list: The open list to return the best element from
        @return: a pair of (operator, successor_state) where the operator is the action applied to reach the successor
        """
        # Convert open_list to a heap based on action failure weights
        heap = []
        counter = 0
        for op, state in open_list:
            heap.append((self.action_failure_weights[op], counter, (op, state)))
            counter += 1
        heapq.heapify(heap)

        # Pop the operator with the lowest action failure weight
        _, _, best_successor = heapq.heappop(heap)
        return best_successor

    def update_ordering_measure(self, successor_node):
        operator_action_failure_weight = self.calc_action_failure_weight(successor_node)
        self.action_failure_weights[successor_node.action] = operator_action_failure_weight

    def calc_action_failure_weight(self, successor_node):
        """
        Calculates the action failure weight for the given operator and state.

        where:
            a is an operator
            s_i is a state
            h(s) is the heuristic value of state s


        f_new(a) = f_old(a) - (h(s_c) - h(s_i) - 1) if (T)
                    f_old(a) otherwise

        afw = action failure weight

        T = (h(s_i) > h(s_c)) and s_i = Result(s_c, a)

        @param successor_node: The successor node to calculate the action failure weight for
        @return: The action failure weight.
        """
        operator = successor_node.action
        parent_node = successor_node.parent

        action_failure_weight = self.action_failure_weights[operator]

        successor_heuristic_value = self.heuristic(successor_node)
        initial_heuristic_value = self.heuristic(parent_node)

        if successor_heuristic_value > initial_heuristic_value:  # I think that maybe this should be < instead of >?
            return action_failure_weight - (successor_heuristic_value - initial_heuristic_value - 1)

        return action_failure_weight

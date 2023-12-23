from ..utils import priority_queue
from .ordering_base import Ordering


class LeastFailedFirst(Ordering):
    """
    This class implements the Least Failed First ordering.
    """

    def __init__(self, heuristic, task):
        super().__init__(heuristic)
        self.action_failure_weights = {}

        for operator in task.operators:
            self.action_failure_weights[operator] = 0

    def __call__(self, successor_list, current_state):
        """
        This method returns the next action to apply to the given state.
        @param successor_list: The open list to use.
        @return: The next action to apply to the given state.
        """
        if not successor_list:
            return None

        best_operator = None

        for successor in successor_list:
            operator, state = successor
            if (best_operator is None or self.action_failure_weights[operator] <
                    self.action_failure_weights[best_operator]):
                best_operator = operator

        return best_operator

    def update(self, successor):
        operator, state = successor
        self.calc_action_failure_weight(operator, state, operator.apply(state))
        return self.action_failure_weights[operator]

    def calc_action_failure_weight(self, operator, initial_state, successor_state):
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

        @param operator: The operator to calculate the action failure weight for.
        @param initial_state: The state to calculate the action failure weight for.
        @param successor_state: The successor state to calculate the action failure weight for.
        @return: The action failure weight.
        """
        current_weight = self.action_failure_weights[operator]

        successor_heuristic_value = self.heuristic(successor_state)
        initial_heuristic_value = self.heuristic(initial_state)

        if successor_heuristic_value > initial_heuristic_value:  # I think that maybe this should be < instead of >?
            return current_weight - (successor_heuristic_value - initial_heuristic_value - 1)

        return current_weight

class Ordering:

    def __init__(self, heuristic):
        self.heuristic = heuristic

    def __call(self, successor_list, current_state):
        """
        This method returns the next action to apply to the given state.
        @param successor_list: the list of successors to use.
        @param current_state: the current state.
        @return: The next action to apply to the given state.
        """
        raise NotImplementedError

    def update(self, successor):
        """
        @param successor: Should contain the operator and the successor state.
        @return: This should be the new ordering measure for the given operator.
        """
        raise NotImplementedError

class Ordering:

    def __init__(self, heuristic):
        self.heuristic = heuristic

    def pop_best_of(self, open_list):
        """
        This method returns the next action to apply to the given state.
        @param open_list: the list of successors to use.
        @return: The next action to apply to the given state.
        """
        raise NotImplementedError

    def update_ordering_measure(self, successor_node):
        """
        @param successor_node: Should contain the operator and the successor state.
        @return: This should be the new ordering measure for the given operator.
        """
        raise NotImplementedError

class LeastFailedFirst():
    """
    This class implements the Least Failed First (LFF) heuristic for operator ordering.
    The LFF heuristic orders operators based on their failure weight, which is calculated
    based on the heuristic value of the states they lead to.
    """

    def __init__(self, planning_task):
        """
        Initializes the LFF heuristic with the given planning task.
        The failure weight for each operator in the task is initially set to 0.
        """
        self.failure_weight = {op: 0 for op in planning_task.operators}

    def calculate_action_failure_weight(self, initial_h_val, operator, successor_h_val):
        """
        Calculates the failure weight for the given operator based on the heuristic values
        of the initial and successor states. If the successor state has a lower heuristic
        value, the failure weight is not increased.
        """
        old_weight = self.failure_weight[operator]
        if initial_h_val > successor_h_val:
            return old_weight  
        return old_weight - (successor_h_val - initial_h_val - 1)
    
    def update_action_failure_weight(self, initial_h_val, operator, successor_h_val):
        """
        Updates the failure weight for the given operator. If the operator is not in the
        failure weight dictionary, it is added with a weight of 0. Then, the failure weight
        is calculated and updated.
        """
        if operator not in self.failure_weight:
            self.failure_weight[operator] = 0
        new_weight = self.calculate_action_failure_weight(initial_h_val, operator, successor_h_val)
        self.failure_weight[operator] = new_weight

    def get_applicable_operators(self, state):
        """
        Returns a list of operators that are applicable in the given state.
        """
        return [op for op in self.failure_weight if op.applicable(state)]
    
    def successor_generator(self, state):
        """
        Returns a generator that yields the operators and successor states for the given state.
        The operators are sorted by their failure weight and name, in descending order.
        """
        applicable_operators = self.get_applicable_operators(state)
        sorted_operators = sorted(applicable_operators, key=lambda x: (self.failure_weight[x], x.name), reverse=True)
        for op in sorted_operators:
            yield (op, op.apply(state))

    def successor_generator_no_ordering(self, state):
        """
        Returns a generator that yields the operators and successor states for the given state,
        without any ordering of the operators.
        """
        applicable_operators = self.get_applicable_operators(state)
        for op in applicable_operators:
            yield (op, op.apply(state))
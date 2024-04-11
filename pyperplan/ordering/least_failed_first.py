
class LeastFailedFirst():
    def __init__(self, planning_task):
        self.failure_weight = {op: 0 for op in planning_task.operators}

    def calculate_action_failure_weight(self, initial_h_val, operator, successor_h_val):
        old_weight = self.failure_weight[operator]
        if initial_h_val > successor_h_val:
            return old_weight  
        return old_weight - (successor_h_val - initial_h_val - 1)
    
    def update_action_failure_weight(self, initial_h_val, operator, successor_h_val):
        # Check if the operator is in the failure weight dictionary
        if operator not in self.failure_weight:
            self.failure_weight[operator] = 0

        # Calculate the new weight
        new_weight = self.calculate_action_failure_weight(initial_h_val, operator, successor_h_val)

        # Update the weight
        self.failure_weight[operator] = new_weight

    def get_applicable_operators(self, state):
        return [op for op in self.failure_weight if op.applicable(state)]
    
    def successor_generator(self, state):
        applicable_operators = self.get_applicable_operators(state)
        # Sort the operators, first by the failure weight and then by the operator name, max to min
        sorted_operators = sorted(applicable_operators, key=lambda x: (self.failure_weight[x], x.name), reverse=True)
        
        # We need to return a generator that yields the operator and the successor state
        for op in sorted_operators:
            yield (op, op.apply(state))

    def successor_generator_no_ordering(self, state):
        applicable_operators = self.get_applicable_operators(state)
        for op in applicable_operators:
            yield (op, op.apply(state))

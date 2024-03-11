from typing import Self
import heapq
from itertools import product
from ..task import Operator, Task

from .relaxation import hFFHeuristic

""" This module contaims the partial delete relaxation heuristic hCFF. """

"""
Definition the Pi^C compilation - (Keyder et al. 2014)

Given a STRIPS planning task P = (F, A, I, G)

- F = fluent set
- A = action set
- I = initial state
- G = goal state

and a set of non-unit conjunctions C subseteq powerset(F) Pi^C us the planning
task (F^C, A^C, I^C, G^C). Where A^C contains an action a^{C'} for each pair
a in A and C' in C such that forall c' in C'

1. del(a) union c' = emptyset wedge add(a) union c' neq emptyset, and
2. forall c in C((c subseteq c' wedge add(a) union c neq emptyset) implies c in C')

and a^{C'} is given by the following:

    del(a^{C'}) = emptyset,
    ce(a^{C'}) = emptyset, and
    pre(a^{C'}) = (pre(a) union PRODUCT_{c' in C'} (c' \ add(a)))^C
    add(a^{C'}) = (add(a) union (pre(a) \ del(a)))^C union { pi_{c'} | c' in C' }
"""

class RelaxedConjOperator():
    def __init__(self, operator: Operator, c_prime: set, c: set):
        self.c = c
        self.operator = self.compile_operator(c_prime, operator)
        
    def compile_operator(self, c_prime: set, operator: Operator)->Operator:
        compiled_preconditions = set(operator.preconditions)
        for conjunction in c_prime:
            compiled_preconditions.update(conjunction - operator.add_effects)
        compiled_preconditions = self.power_set_functions(compiled_preconditions, self.c)
            
        compiled_add_effects = set(operator.add_effects)
        compiled_add_effects.update(compiled_preconditions - operator.del_effects)
        compiled_add_effects = self.power_set_functions(compiled_add_effects, self.c)
        compiled_add_effects.union({conjunction for conjunction in c_prime})
        
        compiled_delete_effects = set()
        compiled_operator = Operator(self.name, compiled_preconditions, compiled_add_effects, compiled_delete_effects)
        
        return compiled_operator
        
    def power_set_functions(self, X: set, Y: set)-> set:
        function_set = set()
        
        for y in Y:
            for x in X:
                def f(y_val):
                    if y_val == y:
                        return x
                    else:
                        return None
                function_set.add(f)
                
        return function_set
    
class hCFFHeuristic(hFFHeuristic):
    def __init__(self, task):
        super().__init__(task)

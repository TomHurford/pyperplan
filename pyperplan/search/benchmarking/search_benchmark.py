from functools import total_ordering
import logging
import time

class Benchmark():
    def __init__(self, task_name, heuristic_name, search_name, lookahead_name, ordering_name):
        self.logger = logging.getLogger(__name__)

        # Max time
        self.max_time = 60

        # Overview Information
        self.task_name = task_name
        self.heuristic_name = heuristic_name
        self.search_name = search_name
        self.lookahead_name = lookahead_name
        self.ordering_name = ordering_name

        # Search statistics
        self.total_expansions = 0
        self.total_heuristic_calls = 0
        self.total_ordering_calls = 0
        self.solution_found = False
        self.solution_length = 0
        self.num_restarts = 0
        self.start_time = 0

        # Lookahead statistics
        self.lookahead_completions = []
        # (lookahead_success: bool, lookahead_expansion_count: int, lookahead_heuristic_call_count: int, lookahead_exit_message: str)

    def restart(self):
        self.num_restarts += 1

    def log_lookahead(self, lookahead_success, lookahead_expansion_count, lookahead_heuristic_call_count, lookahead_ordering_call_count, lookahead_exit_message):
        self.lookahead_completions.append((lookahead_success, lookahead_expansion_count, lookahead_heuristic_call_count, lookahead_ordering_call_count, lookahead_exit_message))

    def start_timer(self):
        self.start_time = time.time()

    def time_up(self):
        return time.time() - self.start_time > self.max_time

    def log_solution(self, solution, message):
        time_passed = time.time() - self.start_time

        if solution != None:
            self.solution_found = True
            self.solution_length = len(solution)

        if message == None:
            # Get the message from the last lookahead completion
            message = self.lookahead_completions[-1][4]

        self.total_expansions = sum([expansions for (lookahead_success, expansions, heuristic_calls, ordering_calls, exit_message) in self.lookahead_completions])

        self.total_heuristic_calls = sum([heuristic_calls for (lookahead_success, expansions, heuristic_calls, ordering_calls, exit_message) in self.lookahead_completions])

        self.total_ordering_calls = sum([ordering_calls for (lookahead_success, expansions, heuristic_calls, ordering_calls, exit_message) in self.lookahead_completions])

        self.logger.benchmarks(','.join([
            str(self.task_name),
            str(self.search_name),
            str(self.heuristic_name),
            str(self.lookahead_name),
            str(self.ordering_name),
            str(self.solution_found),
            str(self.solution_length),
            str(time_passed),
            str(len(self.lookahead_completions)),
            str(self.total_expansions),
            str(self.total_heuristic_calls),
            str(self.total_ordering_calls),
            str(self.num_restarts),
            str(message),
            str(self.lookahead_completions)
        ]))

    

    
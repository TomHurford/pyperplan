from heapq import heappush, heappop, heapify


class HeuristicPriorityQueue():
    pq = []
    entry_finder = {}
    REMOVED = '<removed-task>'
    counter = 0

    def __init__(self):
        heapify(self.pq)

    def add_state(self, state, priority=0):
        if state in self.entry_finder:
            self.remove_state(state)
        count = self.counter
        self.counter += 1
        entry = [priority, count, state]
        self.entry_finder[state] = entry
        heappush(self.pq, entry)

    def remove_state(self, state):
        entry = self.entry_finder.pop(state)
        entry[-1] = self.REMOVED

    def pop(self):
        while self.pq:
            priority, count, state = heappop(self.pq)
            if state is not self.REMOVED:
                del self.entry_finder[state]
                return state
        raise KeyError('pop from an empty priority queue')

    def empty(self):
        return len(self.entry_finder) == 0

    def reset(self):
        self.pq = []
        self.entry_finder = {}
        self.counter = 0

    def __len__(self):
        return len(self.entry_finder)

from heapq import heappush, heappop, heapify


class PriorityQueue:
    pq = []
    entry_finder = {}
    REMOVED = '<removed-task>'
    counter = 0
    count = 0

    def __init__(self):
        heapify(self.pq)

    def add_item(self, item, priority=0):
        if item in self.entry_finder:
            self.remove_item(item)
        count = self.counter
        self.counter += 1
        self.count += 1
        entry = [priority, count, item]
        self.entry_finder[item] = entry
        heappush(self.pq, entry)

    def remove_item(self, state):
        entry = self.entry_finder.pop(state)
        entry[-1] = self.REMOVED
        self.count -= 1

    def pop(self):
        while self.pq:
            priority, count, item = heappop(self.pq)
            if item is not self.REMOVED:
                del self.entry_finder[item]
                self.count -= 1
                return item
        raise KeyError('pop from an empty priority queue')

    def empty(self):
        return self.count == 0

    def reset(self):
        self.pq = []
        self.entry_finder = {}
        self.counter = 0
        self.count = 0

    def __len__(self):
        return self.count

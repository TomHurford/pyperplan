from collections import deque


class Queue:
    def __init__(self):
        self.queue = deque()

    def is_empty(self):
        return len(self.queue) == 0

    def push(self, item):
        self.queue.append(item)

    def pop(self):
        if self.is_empty():
            raise IndexError("Queue is empty")
        return self.queue.popleft()

    def size(self):
        return len(self.queue)

    def list(self):
        return list(self.queue)

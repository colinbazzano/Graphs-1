class Stack():
    def __init__(self):
        self.stack = []

    def push(self, value):
        self.stack.append(value)

    def pop(self):
        if self.size() > 0:
            return self.stack.pop()
        else:
            return None

    def size(self):
        return len(self.stack)


def earliest_ancestor(ancestors, starting_node):
    s = Stack()  # create a stack
    s.push([starting_node])  # add the starting node to the stack

    longest_path = []  # return this

    visited = set()  # visited set

    while s.size() > 0:
        path = s.pop()  # pop it off of the stack
        node = path[-1]  # last item

        if len(path) > len(longest_path):
            # if we have a path longer than the longest_path, make it the new longest path
            longest_path = path

        # return the smaller
        if len(path) == len(longest_path) and path[-1] < longest_path[-1]:
            longest_path = path

        if node not in visited:
            visited.add(node)

            for pair in ancestors:
                if pair[1] == node:
                    new_path = [*path, pair[0]]
                    s.push(new_path)

    if longest_path[-1] == starting_node:
        return -1

    # return the last item in longest path, aka the earliest ancestor
    return longest_path[-1]


test_ancestors = [(1, 3), (2, 3), (3, 6), (5, 6), (5, 7),
                  (4, 5), (4, 8), (8, 9), (11, 8), (10, 1)]

earliest_ancestor(test_ancestors, 6)

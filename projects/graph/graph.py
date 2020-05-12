"""
Simple graph implementation
"""
from util import Stack, Queue  # These may come in handy


class Graph:

    """Represent a graph as a dictionary of vertices mapping labels to edges."""

    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex_id):
        """
        Add a vertex to the graph.
        """
        self.vertices[vertex_id] = set()

    def add_edge(self, v1, v2):
        """
        Add a directed edge to the graph.
        """
        if v1 in self.vertices and v2 in self.vertices:
            # add connection
            self.vertices[v1].add(v2)
        else:
            raise IndexError(
                f"Vertices with the ids of {v1} and {v2} do not exist in this graph")

    def get_neighbors(self, vertex_id):
        """
        Get all neighbors (edges) of a vertex.
        """
        return self.vertices[vertex_id]

    def bft(self, starting_vertex):
        """
        Print each vertex in breadth-first order
        beginning from starting_vertex.
        """
        q = Queue()
        q.enqueue(starting_vertex)
        # keep track of visited nodes
        visited = set()
        while q.size() > 0:
            # dequeue first vert
            v = q.dequeue()
            # if it has not been visited
            if v not in visited:
                print(v)
                visited.add(v)  # mark as visited
                for next_vert in self.get_neighbors(v):
                    q.enqueue(next_vert)

    def dft(self, starting_vertex):
        """
        Print each vertex in depth-first order
        beginning from starting_vertex.
        """
        s = Stack()  # create a stack
        s.push(starting_vertex)
        # keep track of visited nodes
        visited = set()
        while s.size() > 0:  # while items on the stack
            # pop last item
            v = s.pop()
            # if it has not been visited
            if v not in visited:
                print(v)
                visited.add(v)  # mark as visited
                for next_vert in self.get_neighbors(v):
                    s.push(next_vert)

    def dft_recursive(self, starting_vertex):
        """
        Print each vertex in depth-first order
        beginning from starting_vertex.

        This should be done using recursion.
        """
        def dft_helper(vertex, visited):
            if vertex not in visited:
                visited.add(vertex)
                print(vertex)

                for neighbor in self.get_neighbors(vertex):
                    dft_helper(neighbor, visited)
        visited = set()
        dft_helper(starting_vertex, visited)

    def dfs_recursive_lecture(self, start_vert, target_value, visited=None, path=None):
        print("DFS RECURSIVE:", start_vert)

        if visited is None:
            visited = set()

        if path is None:
            path = []

        visited.add(start_vert)

        # Make a copy of the list, adding the new vert on
        path = path + [start_vert]  # list concat, BUT MAKES A NEW LIST

        # base case
        if start_vert == target_value:
            return path  # comes back up through the stack when this is called

        for child_vert in self.vertices[start_vert]:
            if child_vert not in visited:
                new_path = self.dfs_recursive_lecture(
                    child_vert, target_value, visited, path)

                if new_path:
                    return new_path

        return None
        # explore
        # print this node
        # explore all of its unvisited neighbors
        # return

    def dft_recursive_lecture(self, start_vert, visited=None):
        print(start_vert)
        if visited is None:
            visited = set()

        visited.add(start_vert)

        for child_vert in self.vertices[start_vert]:
            if child_vert not in visited:
                self.dft_recursive_lecture(child_vert, visited)

    def bfs(self, starting_vertex, destination_vertex):
        """
        Return a list containing the shortest path from
        starting_vertex to destination_vertex in
        breath-first order.
        """
        queue = Queue()  # creates a queue
        visited = set()
        path = [starting_vertex]  # a list with our starting vertex
        queue.enqueue(path)

        while queue.size() > 0:  # while there's items in the queue
            path = queue.dequeue()  # dequeue
            vertex = path[-1]  # last item in the path

            if vertex == destination_vertex:  # if we have found what we are looking for
                return path
            if vertex not in visited:
                visited.add(vertex)  # add the current vert to the visited
                for neighbor in self.get_neighbors(vertex):
                    new_path = path[:]  # spread in path
                    new_path.append(neighbor)  # append neighbor to new path
                    queue.enqueue(new_path)  # enqueue the neighbors

    def dfs(self, starting_vertex, destination_vertex):
        """
        Return a list containing a path from
        starting_vertex to destination_vertex in
        depth-first order.
        """
        stack = Stack()  # create a stack
        visited = set()
        path = [starting_vertex]
        # add path to the stack (first time through its the starting vert)
        stack.push(path)

        while stack.size() > 0:
            path = stack.pop()  # pop item off the stack
            vertex = path[-1]

            if vertex == destination_vertex:  # we have found it
                return path
            if vertex not in visited:  # if it's not in visited
                visited.add(vertex)  # add it to visited
                # for each neighbor
                for neighbor in self.get_neighbors(vertex):
                    new_path = path[:]  # new path with old path
                    new_path.append(neighbor)  # add neighbors to new path
                    stack.push(new_path)  # add it to the stack

    def dfs_recursive(self, starting_vertex, destination_vertex):
        """
        Return a list containing a path from
        starting_vertex to destination_vertex in
        depth-first order.

        This should be done using recursion.
        """
        def dft_helper(vertex, visited):
            if vertex == destination_vertex:  # base case, we have found it
                return [vertex]

            if vertex not in visited:  # if it's not in visited
                visited.add(vertex)  # add it to visited
                if self.get_neighbors(vertex) is None:  # if there are no neighbors
                    return None
                for neighbor in self.get_neighbors(vertex):
                    # call it recursively on the neighbors
                    path = dft_helper(neighbor, visited)
                    if path:
                        return [vertex, *path]  # return vertex and path items

        visited = set()  # set visited from helper to a set
        path = dft_helper(starting_vertex, visited)

        return path


if __name__ == '__main__':
    graph = Graph()  # Instantiate your graph
    # https://github.com/LambdaSchool/Graphs/blob/master/objectives/breadth-first-search/img/bfs-visit-order.png
    graph.add_vertex(1)
    graph.add_vertex(2)
    graph.add_vertex(3)
    graph.add_vertex(4)
    graph.add_vertex(5)
    graph.add_vertex(6)
    graph.add_vertex(7)
    graph.add_edge(5, 3)
    graph.add_edge(6, 3)
    graph.add_edge(7, 1)
    graph.add_edge(4, 7)
    graph.add_edge(1, 2)
    graph.add_edge(7, 6)
    graph.add_edge(2, 4)
    graph.add_edge(3, 5)
    graph.add_edge(2, 3)
    graph.add_edge(4, 6)

    '''
    Should print:
        {1: {2}, 2: {3, 4}, 3: {5}, 4: {6, 7}, 5: {3}, 6: {3}, 7: {1, 6}}
    '''
    print(graph.vertices)

    '''
    Valid BFT paths:
        1, 2, 3, 4, 5, 6, 7
        1, 2, 3, 4, 5, 7, 6
        1, 2, 3, 4, 6, 7, 5
        1, 2, 3, 4, 6, 5, 7
        1, 2, 3, 4, 7, 6, 5
        1, 2, 3, 4, 7, 5, 6
        1, 2, 4, 3, 5, 6, 7
        1, 2, 4, 3, 5, 7, 6
        1, 2, 4, 3, 6, 7, 5
        1, 2, 4, 3, 6, 5, 7
        1, 2, 4, 3, 7, 6, 5
        1, 2, 4, 3, 7, 5, 6
    '''
    graph.bft(1)

    '''
    Valid DFT paths:
        1, 2, 3, 5, 4, 6, 7
        1, 2, 3, 5, 4, 7, 6
        1, 2, 4, 7, 6, 3, 5
        1, 2, 4, 6, 3, 5, 7
    '''
    graph.dft(1)
    graph.dft_recursive(1)

    '''
    Valid BFS path:
        [1, 2, 4, 6]
    '''
    print(graph.bfs(1, 6))

    '''
    Valid DFS paths:
        [1, 2, 4, 6]
        [1, 2, 4, 7, 6]
    '''
    print(graph.dfs(1, 6))
    print(graph.dfs_recursive(1, 6))

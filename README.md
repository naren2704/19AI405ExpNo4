## ExpNo 4 : Implement A* search algorithm for a Graph
### Name:B.NARENDRAN
### Register Number: 212222240069
## Aim:
## To ImplementA * Search algorithm for a Graph using Python 3.

## Algorithm:
A* Search Algorithm

Initialize the open list

Initialize the closed list put the starting node on the open list (you can leave its f at zero)

while the open list is not empty

a) find the node with the least f on the open list, call it "q"

b) pop q off the open list

c) generate q's 8 successors and set their parents to q

d) for each successor
```
i) if successor is the goal, stop search

ii) else, compute both g and h for successor
  successor.g = q.g + distance between  successor and q
  successor.h = distance from goal to 
  successor (This can be done using many 
  ways, we will discuss three heuristics- 
  Manhattan, Diagonal and Euclidean 
  Heuristics)
  
  successor.f = successor.g + successor.h

iii) if a node with the same position as 
    successor is in the OPEN list which has a 
   lower f than successor, skip this successor

iV) if a node with the same position as 
    successor  is in the CLOSED list which has
    a lower f than successor, skip this successor
    otherwise, add  the node to the open list
```
end (for loop)
     
e) push q on the closed list end (while loop)

## PROGRAM
```
import heapq

class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex, edges):
        self.vertices[vertex] = edges

    def heuristic(self, node, goal):
        # A simple heuristic function, you may modify this according to your problem
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    def a_star_search(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {vertex: float('inf') for vertex in self.vertices}
        g_score[start] = 0
        f_score = {vertex: float('inf') for vertex in self.vertices}
        f_score[start] = self.heuristic(start, goal)

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for neighbor in self.vertices[current]:
                tentative_g_score = g_score[current] + self.vertices[current][neighbor]
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return None
if __name__ == "__main__":
    graph = Graph()
    graph.add_vertex('A', {'B': 5, 'C': 10})
    graph.add_vertex('B', {'A': 5, 'D': 8})
    graph.add_vertex('C', {'A': 10, 'D': 5})
    graph.add_vertex('D', {'B': 8, 'C': 5})

    start = 'A'
    goal = 'D'

    path = graph.a_star_search(start, goal)
    if path:
        print("Path found:", path)
    else:
        print("Path not found")
```

## Sample Graph I

![image](https://github.com/naren2704/19AI405ExpNo4/assets/118706984/1cac66c1-715b-415c-86ff-7a5ae97711f9)

## Sample Input
```
10 14
A B 6
A F 3
B D 2
B C 3
C D 1
C E 5
D E 8
E I 5
E J 5
F G 1
G I 3
I J 3
F H 7
I H 2
A 10
B 8
C 5
D 7
E 3
F 6
G 5
H 3
I 1
J 0
```

## Sample Output

Path found: ['A', 'F', 'G', 'I', 'J']

## Sample Graph II

![image](https://github.com/naren2704/19AI405ExpNo4/assets/118706984/75057795-feb0-46f3-95c0-b0cd329fc219)

## Sample Input
```
6 6
A B 2
B C 1
A E 3
B G 9
E D 6
D G 1
A 11
B 6
C 99
E 7
D 1
G 0
```
## Sample Output

Path found: ['A', 'E', 'D', 'G']

## Result:

Thus the A* implementation is successfull executed.


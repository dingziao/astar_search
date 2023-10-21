from SearchSolution import SearchSolution
from heapq import heappush, heappop

class AstarNode:
    # each search node except the root has a parent node
    # and all search nodes wrap a state object
    def __init__(self, state, heuristic, parent=None, transition_cost=0):
        self.state = state
        self.heuristic = heuristic
        self.parent = parent
        self.transition_cost = transition_cost

    def priority(self):
        return  self.transition_cost + self.heuristic

    # comparison operator,
    # needed for heappush and heappop to work with AstarNodes:
    def __lt__(self, other):
        return self.priority() < other.priority()


# take the current node, and follow its parents back
#  as far as possible. Grab the states from the nodes,
#  and reverse the resulting list of states.
def backchain(node):
    result = []
    current = node
    while current:
        result.append(current.state)
        current = current.parent

    result.reverse()
    return result


def astar_search(search_problem, heuristic_fn):
    start_node = AstarNode(search_problem.start_state, heuristic_fn(search_problem.start_state))
    pqueue = []
    heappush(pqueue, start_node)

    solution = SearchSolution(search_problem, "Astar with heuristic " + heuristic_fn.__name__)

    # record the cost of visited states
    visited_cost = {}
    visited_cost[start_node.state] = 0

    # Record all visited nodes
    close_set = set()

    while len(pqueue):

        # Visit the node with the highest priority
        cur_node = heappop(pqueue)
        close_set.add(cur_node.state)
        solution.nodes_visited += 1
        # If the goal state is found, the path is generated and the solution object is returned.
        if search_problem.meet_goal_state(cur_node.state):
            solution.path = backchain(cur_node)
            solution.cost = cur_node.transition_cost
            return solution

        # Visit all successor states.
        for next_state in search_problem.get_successors(cur_node.state):
            if next_state in close_set:
                continue
            # If have visited
            if next_state in visited_cost:
                # If there is a path with lower cost, update visited_cost
                if visited_cost[cur_node.state] + 1 < visited_cost[next_state]:
                    visited_cost[next_state] = visited_cost[cur_node.state] + 1
                    heappush(pqueue, AstarNode(next_state, heuristic_fn(next_state), cur_node, visited_cost[next_state]))
            # If have not visited, update visited_cost
            else:
                if next_state[1:] == cur_node.state[1:]:
                    visited_cost[next_state] = visited_cost[cur_node.state]
                else:
                    visited_cost[next_state] = visited_cost[cur_node.state] + 1
                heappush(pqueue, AstarNode(next_state, heuristic_fn(next_state), cur_node, visited_cost[next_state]))
    return  -1
# search.py
# ---------

"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    
    """
  #  print("Start:", problem.getStartState())
  #  print("Is the start a goal?", problem.isGoalState(problem.getStartState())) 
  #  print("Start's successors:", problem.getSuccessors(problem.getStartState()))


    # Initialize the stack for the fringe
    fringe = util.Stack()
    # Create a set to keep track of visited nodes
    visited = set()
    # Initialize a list to store the actions
    actionList = []
    
    fringe.push((problem.getStartState(), actionList))

    while not fringe.isEmpty():
        # Pop the node and associated actions from the stack
        node, actions = fringe.pop()
        
        # Check if the node has been visited before
        if node not in visited:
            visited.add(node)  # Mark the node as visited
            
            # Check if the current node is the goal state
            if problem.isGoalState(node):
                return actions

            # Explore the successors of the current node
            for successor in problem.getSuccessors(node):
                coordinate, direction, cost = successor
                nextActions = actions + [direction]
                fringe.push((coordinate, nextActions))
    
    # If no solution is found, return an empty list
    return "No solution founded "

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    fringe = util.Queue()
    # set for tracking visited nodes
    visited = set()
    # actions is stored here 
    actionList = []
    
    fringe.push((problem.getStartState(), actionList))

    while not fringe.isEmpty():
        
        node, actions = fringe.pop()
        
        # Check if the node has been visited before and mark it 
        if node not in visited:
            visited.add(node)  
            
            if problem.isGoalState(node):
                return actions

            # Explore the successors of the current node
            for successor in problem.getSuccessors(node):
                coordinate, direction, cost = successor
                nextActions = actions + [direction]
                fringe.push((coordinate, nextActions))
    
    return "No solution founded"

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    
    fringe = util.PriorityQueue()
    # set for tracking visited nodes
    visited = []
    #action is stored here
    actionList = []
    cost = 0

    fringe.push((problem.getStartState(), actionList, cost), cost)
    while not fringe.isEmpty():
        
        node, actions, cost = fringe.pop()
        # Check if the node has been visited before and mark it
        if node not in visited:
            visited.append(node)  
            
            if problem.isGoalState(node):
                return actions

            # Explore the successors of the current node
            for successor in problem.getSuccessors(node):
                coordinate, direction, step_cost = successor
                nextActions = actions + [direction]
                nextCost = cost + step_cost   ## cost calc 
                fringe.push((coordinate, nextActions, nextCost), nextCost)
    

    return "No solution founded"


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    

    fringe = util.PriorityQueue()
    # set for tracking visited nodes
    visited = set()
    
    # Initialize a list to store the actions
    actions = []
    # Calculate the initial priority using the heuristic estimate
    initial_priority = heuristic(problem.getStartState(), problem)
    # Place the starting point in the priority queue
    fringe.push((problem.getStartState(), actions), initial_priority)
    
    while not fringe.isEmpty():
        # Dequeue the node and associated actions from the priority queue
        node, actions = fringe.pop()
        
        # Check if the node has been visited before
        if node not in visited:
            visited.add(node)  # Mark the node as visited
            
            # Check if the current node is the goal state
            if problem.isGoalState(node):
                return actions  

        
            for successor in problem.getSuccessors(node):
                next_node, direction, step_cost = successor
                next_actions = actions + [direction]
                total_cost = problem.getCostOfActions(next_actions) + heuristic(next_node, problem)
                # Push the next node and the new list of actions onto the queue for further exploration.
                fringe.push((next_node, next_actions), total_cost)
    
    # If no solution is found, return an empty list
    return "No solution founded"

#



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

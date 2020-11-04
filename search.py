# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    # Depth first search always uses a stack data structure coz its last-in first-out
    mySearchStack = util.Stack()
    # array to get the area the pacman has already moved in
    pathWalked = []
    # getting the starting point
    startingPoint = problem.getStartState()
    mySearchStack.push((startingPoint, [], 0))  # initial value in the stack with the starting point

    # a loop to traverse the stack as long as its not empty
    while not mySearchStack.isEmpty():

        # get the current successor point while popping it from the stack
        successor, goal, cost = mySearchStack.pop()
        goalState = problem.isGoalState(successor)

        if successor not in pathWalked:
            # if the successor point has not been looked at then add it to the path already walked upon
            pathWalked.append(successor)

            # get the point's successors
            otherSuccessorPoints = problem.getSuccessors(successor)
            print(otherSuccessorPoints)
            #iterate the points successors
            for x, y, z in otherSuccessorPoints:
                # so now  add it to the stack
                mySearchStack.push((x, goal + [y], z))

        if goalState:
            # if the successor point is the goal, return
            return goal

    # what ever this was, it was already in the code
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # same approach as the depth first search, with changes in mainly the data structure

    # same approach as the Depth first search, but we use a queue because breadth first always uses a queue
    # As per the algorithms and data structures
    mySearchQueue = util.Queue()
    pathWalked = []  # array to get the area the pacman has already moved in
    # getting the starting point
    startingPoint = problem.getStartState()
    mySearchQueue.push((startingPoint, [], 0))

    # a loop to traverse the stack as long as its not empty
    while not mySearchQueue.isEmpty():

        # get the current successor point while popping it from the stack
        successor, goal, cost = mySearchQueue.pop()
        goalState = problem.isGoalState(successor)

        if successor not in pathWalked:
            # if the successor point has not been looked at then add it to the path already walked upon
            pathWalked.append(successor)
            otherSuccessorPoints = problem.getSuccessors(successor)
            for x, y, z in otherSuccessorPoints:
                # so now get successors of the current successor point, and add it to the queue
                mySearchQueue.push((x, goal + [y], z))

        if goalState:
            # if the successor point is the goal, return
            return goal

    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    #same approach as the last two solutions
    # we use a priorityQueue for the uniform cost search, to minimize cost
    # UCS always uses a priority queue from algorithms and data structures

    myUCS = util.PriorityQueue()  # initialize the priority queue
    dictionary = util.Counter()  # introduce a modified dictionary (counter) to keep track of counts for a set of keys.

    pathWalked = []  # array to get the area the pacman has already moved in

    #get the starting point
    startingPoint = problem.getStartState()
    myUCS.push((startingPoint, [], 0),
               dictionary[str(startingPoint[0])])  # push into the queue together with the dictionary of keys

    while not myUCS.isEmpty():  # while queue is not empty
        successor, goal, cost = myUCS.pop()
        goalState = problem.isGoalState(successor)

        if successor not in pathWalked:
            pathWalked.append(successor)
            otherSuccessorPoints = problem.getSuccessors(successor)
            for x, y, z in otherSuccessorPoints:
                # also add the successor points to the dictionary
                dictionary[str(x)] = problem.getCostOfActions(goal + [y])

                # so now add the point to the queue together with the dictionary
                myUCS.push((x, goal + [y], z), dictionary[str(x)])

        if goalState:
            # if the successor point is the goal, return
            return goal

    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    #A priorityQueue goes with Astar search algorithm, so here we are
    mySearchQueue = util.PriorityQueue()
    dictionary = util.Counter() # introduce a modified dictionary (counter) to keep track of counts for a set of keys.
    pathWalked = [] # the path the pacman has already moved through

    startingPoint = problem.getStartState() #get the starting point

    #add the startingPoint to the dictionary as in the ucs, but add a heuristic value to it
    dictionary[str(startingPoint[0])] += heuristic(startingPoint, problem)

    # push into the queue together with the dictionary of keys
    mySearchQueue.push((startingPoint, [], 0), dictionary[str(startingPoint[0])])

    #iterate the queue
    while not mySearchQueue.isEmpty():
        # get the current successor point while popping it from the stack
        successor, action, cost = mySearchQueue.pop()
        goalState = problem.isGoalState(successor) #get the goalstate

        if successor not in pathWalked:
            # if the successor point has not been looked at then add it to the path already walked upon
            pathWalked.append(successor)

            # get the point's successors
            otherSuccessors = problem.getSuccessors(successor)
            for x, y, z in otherSuccessors:
                #get the cost of actions and the heuristicValue
                actionsCost = problem.getCostOfActions(action + [y])
                heuristicValue = heuristic(x, problem)

                #add it to the dictionary for the point's successors
                dictionary[str(x)] = actionsCost + heuristicValue

                #so now add the point to the queue together with the dictionary
                mySearchQueue.push((x, action + [y], z), dictionary[str(x)])

        if goalState:
            # if the successor point is the goal, return
            return action

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

# -*- coding: utf-8 -*-
from heapq import heappush, heappop, heapify
from cell_based_forward_search import CellBasedForwardSearch
from collections import deque
from math import sqrt

# This class implements the Greedy - or best first search - planning
# algorithm. It works by using a priority queue: cells are sorted and
# the best cell is put onto the front of the queue, 
# The best current cell is then popped from the front of the queue 

class GREEDYPlanner(CellBasedForwardSearch):

    # self implements a simple GREEDY search algorithm
    
    def __init__(self, title, occupancyGrid, goal):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.goal = goal
        self.priorityQueue = []
        heapify(self.priorityQueue)

    def eucliddistance(self, cell):
        eucliddistance = sqrt(((cell.coords[0]-self.goal[0])**2)+((cell.coords[1]-self.goal[1])**2))    
        return eucliddistance


    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        cost = eucliddistance(cell)
        cell_with_cost = (cost, cell)
        heappush(self.priorityQueue, cell_with_cost )

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.priorityQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = heappop(self.priorityQueue)
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass

 

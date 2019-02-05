# -*- coding: utf-8 -*-
from heapq import heappush, heappop, heapify
from cell_based_forward_search import CellBasedForwardSearch
from collections import deque
from math import sqrt
from time import sleep

# This class implements the Greedy - or best first search - planning
# algorithm. It works by using a priority queue: cells are sorted and
# the best cell is put onto the front of the queue, 
# The best current cell is then popped from the front of the queue 

class DIJKSTRAPlanner(CellBasedForwardSearch):

    # self implements a simple GREEDY search algorithm
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.priorityQueue = []
        heapify(self.priorityQueue)

    # Calculate the euclidean distance between the current cell and the goal   
    def eucliddistance(self, cell):
        eucliddistance = sqrt(((cell.coords[0]-self.goal.coords[0])**2)+((cell.coords[1]-self.goal.coords[1])**2))    
        return eucliddistance

    def pathcostcalc(self,cell):
        #The travel cost from the current cell back to the start
        travelCost = 0
        #Stores the cell into inital 
        initialcell = cell
        
        #Works out the cost using a loop going through the current cells path and adding the distance between each cell
        travelCost = self.computeLStageAdditiveCost(cell.parent, cell)
        if travelCost > 0:
            travelCost += self.eucliddistance(cell)

        while (cell is not None):
            travelCost = travelCost + self.computeLStageAdditiveCost(cell.parent, cell)
            cell = cell.parent

        #Stores into the pathcost into the cell object
        initialcell.pathCost = travelCost

    #Sort the elements and put the best at the front
    def pushCellOntoQueue(self, cell):
        self.pathcostcalc(cell)
        cell_with_cost = (cell.pathCost, cell)
        heappush(self.priorityQueue, cell_with_cost)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.priorityQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell_with_cost = heappop(self.priorityQueue)
        # Get the cell from the cell_with_cost tuple
        cell = cell_with_cost[1]
        return cell

    def resolveDuplicate(self, cell, parentCell):
        newqueue =[]
        
        #Calculate the length between the current cell and the duplicate cell 
        length = self.computeLStageAdditiveCost(cell,parentCell)
        #The predicted path cost of the new path of the duplicate cell
        predicted_path_cost = parentCell.pathCost + length
        
        #If the predicted path is less than the current path then it needs to be changed.
        if predicted_path_cost < cell.pathCost:
            #The duplicate cell needs to have the current cell as its parent
            cell.parent = parentCell
            cell.pathCost = predicted_path_cost

            #Need to remove the cell and replace it with the correct path cost
            currentcell = self.popCellFromQueue()
            while(currentcell.coords != cell.coords):
                newqueue.append(currentcell)
                currentcell = self.popCellFromQueue()
            
            #Stores the cells before it can access the wanted cell then places the new cell and all the old ones
            self.pushCellOntoQueue(cell)
            for oldcells in newqueue:
                self.pushCellOntoQueue(oldcells)
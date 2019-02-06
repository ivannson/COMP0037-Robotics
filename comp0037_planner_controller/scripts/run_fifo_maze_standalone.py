#! /usr/bin/env python

# Import the needed types.
from comp0037_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_planner_controller.fifo_planner import FIFOPlanner

# Create the occupancy grid. Syntax is: number of cells in X, number of cells in Y,
# length of each cell in m
occupancyGrid = OccupancyGrid(21, 21, 0.5)

# The cells are indexed starting from 0.
# Set the state of the cells in the range [11,1]-[11,19] to be occupied.
# This corresponds to the "easy case" in the lectures

for y in xrange(3, 20):
    occupancyGrid.setCell(3, y, 1)
    occupancyGrid.setCell(18, y, 1)

for y in xrange(6,17):
    occupancyGrid.setCell(15, y, 1)

for y in xrange(3,17):
    occupancyGrid.setCell(6, y, 1)

for y in xrange(9,14):
    occupancyGrid.setCell(9, y, 1)
    occupancyGrid.setCell(12, y, 1)

for y in xrange(6,14):
    occupancyGrid.setCell(9, y, 1)

occupancyGrid.setCell(3, 3, 1)

for x in xrange(3,19):
    occupancyGrid.setCell(x, 20, 1)

for x in xrange(7,19):
    occupancyGrid.setCell(x, 3, 1)

for x in xrange(6,16):
    occupancyGrid.setCell(x, 17, 1)

for x in xrange(9,15):
    occupancyGrid.setCell(x, 6, 1)

for x in xrange(9,13):
    occupancyGrid.setCell(x, 14, 1)

start = (0, 0)
goal = (10, 12)
# Create the planner. The first field is the title which will appear in the
# graphics window, the second the occupancy grid used.
planner = FIFOPlanner('Depth First Search', occupancyGrid);

# This causes the planner to slow down and pause for things like key entries
planner.setRunInteractively(True)

# This specifies the height of the window drawn showing the occupancy grid. Everything
# should scale automatically to properly preserve the aspect ratio
planner.setWindowHeightInPixels(400)

# Search and see if a path can be found. Returns True if a path from the start to the
# goal was found and False otherwise
goalReached = planner.search(start, goal)

# Extract the path. This is based on the last search carried out.
path = planner.extractPathToGoal()

# Note that you can run multiple planners - each one will create and update its own window.
# See the minkowski_sum_tester asn an example

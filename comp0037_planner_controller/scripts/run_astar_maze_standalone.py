#! /usr/bin/env python

from comp0037_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_planner_controller.ASTAR_planner import ASTARPLANNER
import sys

occupancyGrid = OccupancyGrid(21, 21, 0.5)

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

# Choice of heuristic is taken as a command line argument
heuristic = sys.argv[1]
weighting = float(sys.argv[2])


planner = ASTARPLANNER('Depth First Search', occupancyGrid);
planner.heuristic = heuristic
planner.weighting = weighting
planner.setRunInteractively(True)

planner.setWindowHeightInPixels(400)

goalReached = planner.search(start, goal)

path = planner.extractPathToGoal()

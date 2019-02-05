#! /usr/bin/env python
import sys
# See run_fifo_standalone.py for documentation. The only difference is that
# a LIFO planner is created instead of a FIFO planner.

from comp313p_planner_controller.occupancy_grid import OccupancyGrid
from comp313p_planner_controller.ASTAR_planner import ASTARPLANNER
import sys

occupancyGrid = OccupancyGrid(21, 21, 0.5)

for x in xrange(3, 15):
    occupancyGrid.setCell(x, 10, 1)

for x in xrange(16,19):
    occupancyGrid.setCell(x,8,1)

for y in range(8,11):
    occupancyGrid.setCell(15,y,1)

start = (3, 10)
goal = (17, 9)

heuristic = sys.argv[1]
print(heuristic)

planner = ASTARPLANNER('Depth First Search', occupancyGrid, heuristic);
planner.setRunInteractively(True)

planner.setWindowHeightInPixels(400)

goalReached = planner.search(start, goal)

path = planner.extractPathToGoal()
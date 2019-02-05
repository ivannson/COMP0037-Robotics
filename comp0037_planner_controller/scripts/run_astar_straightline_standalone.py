#! /usr/bin/env python
import sys
# See run_fifo_standalone.py for documentation. The only difference is that
# a LIFO planner is created instead of a FIFO planner.

from comp313p_planner_controller.occupancy_grid import OccupancyGrid
from comp313p_planner_controller.ASTAR_planner import ASTARPLANNER
import sys

occupancyGrid = OccupancyGrid(21, 21, 0)

start = (5, 10)
goal = (15, 10)

heuristic = sys.argv[1]
print(heuristic)

planner = ASTARPLANNER('Depth First Search', occupancyGrid, heuristic);
planner.setRunInteractively(True)

planner.setWindowHeightInPixels(400)

goalReached = planner.search(start, goal)

path = planner.extractPathToGoal()
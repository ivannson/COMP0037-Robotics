#! /usr/bin/env python



from comp0037_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_planner_controller.ASTAR_planner import ASTARPLANNER
import sys

occupancyGrid = OccupancyGrid(21, 21, 0.5)

for y in xrange(1, 19):
    occupancyGrid.setCell(11, y, 1)

start = (3, 18)
goal = (20, 0)


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

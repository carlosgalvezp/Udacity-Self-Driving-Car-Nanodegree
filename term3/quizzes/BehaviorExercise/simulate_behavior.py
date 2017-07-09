#!/usr/bin/env python

from road import Road
import time

# impacts default behavior for most states
SPEED_LIMIT       = 10 

# all traffic in lane (besides ego) follow these speeds
LANE_SPEEDS       = [6,7,8,9] 

# Number of available "cells" which should have traffic
TRAFFIC_DENSITY   = 0.15

# At each timestep, ego can set acceleration to value between 
# -MAX_ACCEL and MAX_ACCEL
MAX_ACCEL         = 2

# s value and lane number of goal.
GOAL              = (300, 3)

# These affect the visualization
FRAMES_PER_SECOND = 4
AMOUNT_OF_ROAD_VISIBLE = 40

def main():
	road = Road(SPEED_LIMIT, TRAFFIC_DENSITY, LANE_SPEEDS)
	road.update_width = AMOUNT_OF_ROAD_VISIBLE
	road.populate_traffic()
	ego_config = config = {
	    'speed_limit' : SPEED_LIMIT,
	    'num_lanes' : len(LANE_SPEEDS),
	    'goal' : (300, 3),
	    'max_acceleration': MAX_ACCEL
	}
	road.add_ego(2,0, ego_config)
	timestep = 0
	while road.get_ego().s <= GOAL[0]:
		timestep += 1
		if timestep > 150: 
			print "Taking too long to reach goal. Go faster!"
			break
		road.advance()
		print road
		time.sleep(float(1.0) / FRAMES_PER_SECOND)
	ego = road.get_ego()
	if ego.lane == GOAL[1]:
		print "You got to the goal in {} seconds!".format(timestep)
	else:
		print "You missed the goal. You are in lane {} instead of {}.".format(ego.lane, GOAL[1])


if __name__ == "__main__":
	main()

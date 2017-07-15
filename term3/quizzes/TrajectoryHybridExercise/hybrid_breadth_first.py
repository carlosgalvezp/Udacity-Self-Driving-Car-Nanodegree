from math import pi, sqrt, sin, cos, tan, floor
from matplotlib import pyplot as plt

NUM_THETA_CELLS = 90
SPEED = 1.45
LENGTH = 0.5

def theta_to_stack_number(theta):
	"""
	Takes an angle (in radians) and returns which "stack" in the 3D configuration space
	this angle corresponds to. Angles near 0 go in the lower stacks while angles near 
	2 * pi go in the higher stacks.
	"""
	new_theta = (theta + 2 * pi) % (2 * pi)
	stack_number = int(round(new_theta * NUM_THETA_CELLS / (2*pi))) % NUM_THETA_CELLS
	return stack_number

def idx(float_num):
	"""
	Returns the index into the grid for continuous position. So if x is 3.621, then this
	would return 3 to indicate that 3.621 corresponds to array index 3.
	"""
	return int(floor(float_num))

def search(grid, start, goal):
	"""
	Working Implementation of breadth first search. Does NOT use a heuristic
	and as a result this is pretty inefficient. Try modifying this algorithm 
	into hybrid A* by adding heuristics appropriately.
	"""
	closed = [[[0 for row in range(len(grid[0]))] for col in range(len(grid))] for stack in range(NUM_THETA_CELLS)]
	came_from = [[[0 for row in range(len(grid[0]))] for col in range(len(grid))] for stack in range(NUM_THETA_CELLS)]
	x,y,theta = start
	stack = theta_to_stack_number(theta)
	g = 0
	closed[stack][idx(x)][idx(y)] = (g,x,y,theta)
	came_from[stack][idx(x)][idx(y)] = (g,x,y,theta)
	total_closed = 1
	opened = [(g,x,y,theta)]
	while len(opened) > 0:
		opened.sort(reverse=True)
		next = opened.pop()
		g,x,y,theta = next
		test = (idx(x),idx(y))
		# print "testing if {} is == {}".format(test, goal)
		if (idx(x),idx(y)) == goal:
			print "\n###############\nfound path to goal in {} expansions\n".format(total_closed)
			return closed, came_from, (g,x,y,theta)
		for next_state in expand(next):
			g2, x2, y2, theta2 = next_state
			if x2 < 0 or x2 >= len(grid) or y2 < 0 or y2 >= len(grid[0]):
				# invalid cell
				continue
			stack2 = theta_to_stack_number(theta2)
			# try:
			# print "as indices...: {}, {}, {}".format(idx(x2), idx(y2), stack2)  
			# print "closed dims {} x {} x {}".format(len(closed), len(closed[0]), len(closed[0][0]))
			if closed[stack2][idx(x2)][idx(y2)] == 0 and grid[idx(x2)][idx(y2)] == 0:
				
				opened.append((g2, x2, y2, theta2))
				closed[stack2][idx(x2)][idx(y2)] = next_state
				came_from[stack2][idx(x2)][idx(y2)] = next
				total_closed += 1
			# except:
			# 	print "ERROR"
			# 	print "x2, y2, theta2: {}, {}, {}".format(x2,y2,theta2)
			# 	print "as indices...: {}, {}, {}".format(idx(x2), idx(y2), stack2)  
	print "no valid path."
	return closed, came_from, (g,x,y,theta)

def reconstruct_path(came_from, goal, start, final):
	path = [(final)]
	g, x, y, theta = final
	stack = theta_to_stack_number(theta)
	current = came_from[stack][idx(x)][idx(y)]
	g, x, y, theta = current
	stack = theta_to_stack_number(theta)
	while (x,y) != (start[0], start[1]):
		path.append(current)
		# print "came from is {}".format(came_from)
		# print "stack, x, y: {}, {}, {}".format(stack, x, y)
		current = came_from[stack][idx(x)][idx(y)]
		g, x, y, theta = current
		stack = theta_to_stack_number(theta)
	return path

def expand(state):
	g, x, y, theta = state
	g2 = g+1
	next_states = []
	for delta in range(-35, 40, 5):
		delta = pi / 180.0 * delta
		omega = SPEED / LENGTH * tan(delta)
		theta2 = theta + omega
		x2 = x + SPEED * cos(theta2)
		y2 = y + SPEED * sin(theta2)
		next_states.append((g2, x2, y2, theta2))
	return next_states

def show_path(path, start, goal):
    path.reverse()
    X = [p[1] for p in path]
    Y = [p[2] for p in path]
    THETA = [p[3] for p in path]
    plt.scatter(X,Y, color='black')
    plt.scatter([start[0]], [start[1]], color='blue')
    plt.scatter([goal[0]], [goal[1]], color='red')
    plt.show()

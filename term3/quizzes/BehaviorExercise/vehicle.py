class Vehicle(object):
  L = 1
  preferred_buffer = 6 # impacts "keep lane" behavior.

  def __init__(self, lane, s, v, a):
    self.lane = lane
    self.s = s 
    self.v = v
    self.a = a
    self.state = "CS"
    self.max_acceleration = None

  # TODO - Implement this method.
  def update_state(self, predictions):
    """
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    """
    if self.lane < self.goal_lane:
        self.state = 'LCL'
    elif self.lane > self.goal_lane:
        self.state = 'LCR'
    else:
        self.state = "KL" # this is an example of how you change state.

  def configure(self, road_data):
    """
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    """
    self.target_speed = road_data['speed_limit']
    self.lanes_available = road_data["num_lanes"]
    self.max_acceleration = road_data['max_acceleration']
    goal = road_data['goal']
    self.goal_s = goal[0]
    self.goal_lane = goal[1]

  def __repr__(self):
    s = "s:    {}\n".format(self.s)
    s +="lane: {}\n".format(self.lane)
    s +="v:    {}\n".format(self.v)
    s +="a:    {}\n".format(self.a)
    return s

  def increment(self, dt=1):
    self.s += self.v * dt
    self.v += self.a * dt

  def state_at(self, t):
    """
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    """
    s = self.s + self.v * t + self.a * t * t / 2
    v = self.v + self.a * t
    return self.lane, s, v, self.a

  def collides_with(self, other, at_time=0):
    """
    Simple collision detection.
    """
    l,   s,   v,   a = self.state_at(at_time)
    l_o, s_o, v_o, a_o = other.state_at(at_time)
    return l == l_o and abs(s-s_o) <= L 

  def will_collide_with(self, other, timesteps):
    for t in range(timesteps+1):
      if self.collides_with(other, t):
        return True, t
    return False, None

  def realize_state(self, predictions):
    """
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    """
    state = self.state
    if   state == "CS"  : self.realize_constant_speed()
    elif state == "KL"  : self.realize_keep_lane(predictions)
    elif state == "LCL" : self.realize_lane_change(predictions, "L")
    elif state == "LCR" : self.realize_lane_change(predictions, "R")
    elif state == "PLCL": self.realize_prep_lane_change(predictions, "L")
    elif state == "PLCR": self.realize_prep_lane_change(predictions, "R")

  def realize_constant_speed(self):
    self.a = 0

  def _max_accel_for_lane(self, predictions, lane, s):
    delta_v_til_target = self.target_speed - self.v
    max_acc = min(self.max_acceleration, delta_v_til_target)
    in_front = [v for (v_id, v) in predictions.items() if v[0]['lane'] == lane and v[0]['s'] > s ]
    if len(in_front) > 0:
      leading = min(in_front, key=lambda v: v[0]['s'] - s)
      next_pos = leading[1]['s']
      my_next = s + self.v
      separation_next = next_pos - my_next
      available_room = separation_next - self.preferred_buffer
      max_acc = min(max_acc, available_room)
    return max_acc

  def realize_keep_lane(self, predictions):
    self.a = self._max_accel_for_lane(predictions, self.lane, self.s)

  def realize_lane_change(self, predictions, direction):
    delta = -1
    if direction == "L": delta = 1
    self.lane += delta
    self.a = self._max_accel_for_lane(predictions, self.lane, self.s)

  def realize_prep_lane_change(self, predictions, direction):
    delta = -1
    if direction == "L": delta = 1
    lane = self.lane + delta
    ids_and_vehicles = [(v_id, v) for (v_id, v) in predictions.items() if v[0]['lane'] == lane and v[0]['s'] <= self.s]
    if len(ids_and_vehicles) > 0:
      vehicles = [v[1] for v in ids_and_vehicles]

      nearest_behind = max(ids_and_vehicles, key=lambda v: v[1][0]['s'])

      print("nearest behind : {}".format(nearest_behind))
      nearest_behind = nearest_behind[1]
      target_vel = nearest_behind[1]['s'] - nearest_behind[0]['s']
      delta_v = self.v - target_vel
      delta_s = self.s - nearest_behind[0]['s']
      if delta_v != 0:
        print("delta_v {}".format(delta_v))
        print("delta_s {}".format(delta_s))
        time = -2 * delta_s / delta_v
        if time == 0:
          a = self.a
        else:
          a = delta_v / time
        print("raw a is {}".format(a))
        if a > self.max_acceleration: a = self.max_acceleration
        if a < -self.max_acceleration: a = -self.max_acceleration
        self.a = a
        print("time : {}".format(time))
        print("a: {}".format(self.a))
      else :
        min_acc = max(-self.max_acceleration, -delta_s)
        self.a = min_acc

  def generate_predictions(self, horizon=10):
    predictions = []
    for i in range(horizon):
      lane, s, v, a = self.state_at(i)
      predictions.append({'s':s, 'lane': lane})
    return predictions

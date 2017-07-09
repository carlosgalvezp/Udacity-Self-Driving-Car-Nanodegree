import random
from vehicle import Vehicle

class Road(object):
  update_width = 70
  ego_rep = " *** "
  ego_key = -1
  def __init__(self, speed_limit, traffic_density, lane_speeds):
    self.num_lanes = len(lane_speeds)
    self.lane_speeds = lane_speeds
    self.speed_limit = speed_limit
    self.density = traffic_density
    self.camera_center = self.update_width / 2
    self.vehicles = {}
    self.vehicles_added = 0
        
  def get_ego(self):
    return self.vehicles[self.ego_key]
    
  def populate_traffic(self):
    start_s = max(self.camera_center - (self.update_width / 2), 0)
    for l in range(self.num_lanes):
      lane_speed = self.lane_speeds[l]
      vehicle_just_added = False
      for s in range(start_s, start_s + self.update_width):
        if vehicle_just_added:
          vehicle_just_added = False
          continue
        if random.random() < self.density:
          vehicle = Vehicle(l, s, lane_speed, 0)
          vehicle.state = "CS"
          self.vehicles_added += 1
          self.vehicles[self.vehicles_added] = vehicle
          vehicle_just_added = True

  def advance(self):
    predictions = {}
    for v_id, v in self.vehicles.items():
      preds = v.generate_predictions()
      predictions[v_id] = preds
    for v_id, v in self.vehicles.items():
      if v_id == self.ego_key:
        v.update_state(predictions)
        v.realize_state(predictions)
      v.increment()

  def add_ego(self, lane_num, s, config_data):
    for v_id, v in self.vehicles.items():
      if v.lane == lane_num and v.s == s:
        del self.vehicles[v_id]
    ego = Vehicle(lane_num, s, self.lane_speeds[lane_num], 0)
    ego.configure(config_data)
    ego.state = "KL"
    self.vehicles[self.ego_key] = ego

  def cull(self):
    ego = self.vehicles[self.ego_key]
    center_s = ego.s
    claimed = set([(v.lane, v.s) for v in self.vehicles.values()])
    for v_id, v in self.vehicles.items():
      if v.s > (center_s + self.update_width / 2) or v.s < (center_s - self.update_width / 2):
        try:
          claimed.remove((v.lane,v.s))
        except:
          continue
        del self.vehicles[v_id]
        
        placed = False
        while not placed:
          lane_num = random.choice(range(self.num_lanes))
          ds = random.choice(range(self.update_width/2-15,self.update_width/2 -1 ))
          if lane_num > self.num_lanes / 2:
            ds *= -1
          s = center_s + ds
          if (lane_num, s) not in claimed:
            placed = True
            speed = self.lane_speeds[lane_num]
            vehicle = Vehicle(lane_num, s, speed, 0)
            self.vehicles_added += 1
            self.vehicles[self.vehicles_added] = vehicle
            print 'adding vehicle {} at lane {} with s={}'.format(self.vehicles_added, lane_num, s)

  def __repr__(self):
    s = self.vehicles.get(self.ego_key).s
    self.camera_center = max(s, self.update_width / 2)
    s_min = max(self.camera_center - self.update_width /2, 0)
    s_max = s_min + self.update_width
    road = [["     " if i % 3 == 0 else "     "for ln in range(self.num_lanes)] for i in range(self.update_width)]
    for v_id, v in self.vehicles.items():
      if s_min <= v.s < s_max:
        if v_id == self.ego_key:
          marker = self.ego_rep
        else:
          marker = " %03d " % v_id
        road[v.s - s_min][v.lane] = marker
    s = ""
    i = s_min
    for l in road:
      if i % 20 == 0:
        s += "%03d - " % i
      else:
        s += "      "
      i += 1
      s += "|" + "|".join(l) + "|"
      s += "\n"
    return s
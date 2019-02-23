import math

class RADARDetection:
   def __init__(self):
      self.theta = 0
      self.phi = 0
      self.r = 0
      self.rcs = 0
      self.pdh = 0
      self.snr = 0
      self.vrel = 0

      self.long_range = False

   def __lt__(self, other):
      return self.snr < other.snr

   def distancetoother(self, other):
      my_y = math.sin(self.theta) * self.r
      my_x = math.cos(self.theta) * self.r
      other_y = math.sin(other.theta) * other.r
      other_x = math.cos(other.theta) * other.r
      y_distance = my_y - other_y
      x_distance = my_x - other_x
      distance = math.sqrt( math.pow(y_distance,2) + math.pow(x_distance, 2) )

      return distance
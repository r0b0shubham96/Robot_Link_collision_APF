from numpy.linalg import norm

# Class for Go to Goal algorithm (Attractive Potential)
class AttractivePotential(object):
   

    def __init__(self, x_start, x_target):
        
        # Set target
        self.x_target = x_target

        # Attractive Potential Coefficient
        self.K_att = 0.2

        # Initialize Goal distance
        self.updateState(x_start)

    def updateState(self, current):
        
        # Goal distance vector
        self.toGoal = self.x_target - current

        # Goal distance
        self.distanceGoal = norm(self.toGoal)

    def plan(self):
       
        # Attractive potential
        return self.K_att * self.toGoal / self.distanceGoal

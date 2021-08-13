import numpy as np
import pybullet as p
from numpy.linalg import norm
from GoToGoal import AttractivePotential

# Class for obstacle algorithm
class ObstacleAlgorithm(object):
   

    def __init__(self, x_start, x_target):
       

        # Location of obstacle
        self.x_obs = np.array(x_start) + 0.5 * (np.array(x_target) - np.array(x_start))

        # Location of target
        self.x_target = x_target

        # Minimum distance to obstacle for avoidance
        self.d_min = .3

        # Initialize experiment time
        self._t = None

    def updateState(self, current):
        

        # Vector from Obstacle to current state
        self.toObstacle = self.x_obs - current

        # Distance value
        self.distanceObstacle = norm(self.toObstacle)

        # Vector from Current state to Goal
        self.toGoal = self.x_target - current

        # Distance value
        self.distanceGoal = norm(self.toGoal)


# Class for Repulsive potential field algorithm
class RepulsivePotentialAlgorithm(ObstacleAlgorithm):
   

    def __init__(self, x_start, x_target):
        

        # Set obstacle avoidance
        super().__init__(x_start, x_target)

        # Repulsive Potential Coefficient
        self.K_rep = 0.1

        # Set algorithm based parameters
        self.updateState(x_start)

    def getStatus(self):
       

        return self.distanceObstacle < self.d_min

    def plan(self):
        
     
        # Dxxo = self.distanceObstacle

        tangent = self.K_rep/(self.distanceObstacle**2)
        tangent *= ((1/self.distanceObstacle) - (1/self.d_min))
        tangent *= self.toObstacle / self.distanceObstacle

        return tangent
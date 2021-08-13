import pybullet as p
import time
import math
import pickle
from datetime import datetime
import pybullet_data
import numpy as np
import sys
import copy
from numpy.linalg import norm, pinv
from GoToGoal import AttractivePotential
from ObstacleAvoidance import ObstacleAlgorithm, \
                                RepulsivePotentialAlgorithm


# Class for the Kuka IIWA Experiment
class KukaRobotExperiment(object):
  
    # Set timestep in simulation
    dt = 1/240.

    def __init__(self,  debug = True):
        

        # Connect to PyBullet simulator
        self.clid = p.connect(p.SHARED_MEMORY)
        if self.clid < 0:
            p.connect(p.GUI)

        # Set PyBullet installed Data path for URDFs
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load objects
        self.loadObjects()

        # Obtain hardcoded limits, ranges and coefficents
        self.setRobotLimitsRanges()

        # Initialize Robot to rest position
        self.setJointStates(self.rp)

        # Hardcoded value for rest position of Kuka
        self.prevPose = [0, 0, 0]
        self.hasPrevPose = False

        self.trajectory = []

        # Initialize states
        self.initializeParamsAndState()

        # Set Go to goal algorithm
        self.attractivePotential = AttractivePotential(self.x_start, self.x_target)

        # Set algorithm for Obstacle avoidance
        self.obstacleAlgorithm = RepulsivePotentialAlgorithm(self.x_start, self.x_target)

        # Conduct experiment
        self.experiment(debug)

        # Show experiment results
        self.experimentResults()

    def loadObjects(self):
       

        # Load floor plane at -2
        p.loadURDF("plane.urdf",[0,0,-2])

        # Load Robot
        self.robot = p.loadURDF("kuka_iiwa/model.urdf",[0,0,0])
        p.resetBasePositionAndOrientation(
            self.robot,
            [0, 0, 0],
            [0, 0, 0, 1]
        )

        # Joints and End effector Index
        self.robotNumJoints = p.getNumJoints(self.robot)
        self.robotEndEffectorIndex = 6
        assert self.robotNumJoints == 7, "Model incorrect"

        #obstacle sphere
        self.sphereobj = p.loadURDF("sphereobs.urdf", [-0.4 , -0.05,  0.45], useFixedBase = 1)
        #p.changeDynamics(self.sphereobj,-1,radius = 0.2)

        # Camera adjustment
        p.resetDebugVisualizerCamera(
            cameraDistance = 3,
            cameraYaw = 230,
            cameraPitch = -22,
            cameraTargetPosition = [0,0,0]
        )

        # Gravity setting
        p.setGravity(0, 0, 0)

        # Is Simulation Real Time?
        p.setRealTimeSimulation(0)

    def setRobotLimitsRanges(self):
       

        # lower limits for null space
        self.ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]

        # upper limits for null space
        self.ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]

        # joint ranges for null space
        self.jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]

        # restposes for null space
        self.rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]

        # joint damping coefficents
        self.jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    def updateState(self, updateJ = False):
       

        # Get link state
        linkState = p.getLinkState(
                        self.robot,
                        self.robotEndEffectorIndex,
                        computeLinkVelocity = 1,
                        computeForwardKinematics = 1
        )

        # Save x value and find q
        self.x = linkState[4]
        self.q = self.ik(self.x)
        self.q_list.append(self.q)
        
        # Calculate Jacobian
        if updateJ:
            J, _ = p.calculateJacobian(
                        bodyUniqueId = self.robot,
                        linkIndex = self.robotEndEffectorIndex,
                        localPosition = list(linkState[2]),
                        objPositions = list(self.q),
                        objVelocities = [0.] * len(list(self.q)),
                        objAccelerations = [0.] * len(list(self.q))
            )

            self.J = np.array(J)


    def setJointStates(self, q):
       

        # Set each joint's states
        for jointNumber in range(self.robotNumJoints):
            p.resetJointState(self.robot, jointNumber, float(q[jointNumber]))

    def ik(self, x):
       

        q = p.calculateInverseKinematics(
                        bodyUniqueId = self.robot,
                        endEffectorLinkIndex = self.robotEndEffectorIndex,
                        targetPosition = list(x),
                        lowerLimits = self.ll,
                        upperLimits = self.ul,
                        jointRanges = self.jr,
                        restPoses = self.rp
                        )

        return np.array(q)


    def initializeParamsAndState(self):
       
        # Trail debug line delay
        self.trailDuration = 15

        # Start state
        self.x_start = np.array([-.4, -.3, .7])

        # Set current state to start state
        self.x = copy.deepcopy(self.x_start)
        self.q = self.ik(self.x_start)

        # Update states on robot
        self.setJointStates(self.q)

        # Target state
        self.x_target = np.array([-.4, .2, .2])
        self.q_list = []

        # Initialize time
        self.t = 0


    def setRobotTaskReference(self, x_dot_ref):
        

        # Task to Joint Reference
        q_dot_ref = pinv(self.J) @ x_dot_ref

        # Set joint reference for each joint
        for robotJoint in range(self.robotNumJoints):
            p.setJointMotorControl2(
                bodyIndex = self.robot,
                jointIndex = robotJoint,
                controlMode = p.VELOCITY_CONTROL,
                targetVelocity = float(q_dot_ref[robotJoint]),
                force = 500,
                positionGain = 0.03,
                velocityGain = 0.01
            )


    def experiment(self, debug):
       

        # Continue motion until target is reached OR if time exceeds 90s
        while norm(self.x - self.x_target) > 1e-3 and self.t < 90 / self.dt:
            
            self.trajectory.append(self.x)
            # Update timestep
            self.t += self.dt

            # Step simulation seems to not pause the simulation for dt time
            # when debug is false. Corrects that term here.
            if not debug:
                time.sleep(self.dt)

            # Perform simulation in this step
            p.stepSimulation()


            # Obtain robot state by updating the parameters from measurements.
            self.updateState(updateJ = True)
            self.obstacleAlgorithm.updateState(self.x)
            self.attractivePotential.updateState(self.x)


            # HYBRID ALGORITHM:
            # If Obstacle is not reached, Attractive Potential is used
            # If the Obstacle is reached, Attractive potential alongwith Obstacle Algorithm is used
            if self.obstacleAlgorithm.getStatus():

                # Perform planning and obtain the reference velocity
                x_dot_ref = self.attractivePotential.plan() - self.obstacleAlgorithm.plan()
                
                # Draw line for tangents to reference velocity while debugging
                if debug:
                    p.addUserDebugLine(self.x, list(self.x + 4 * x_dot_ref), [0, 1, 1], 5, 2)

            else:

                # Perform planning and obtain the reference velocity
                x_dot_ref = self.attractivePotential.plan()

            # Move the robot joints based on given reference velocity.
            self.setRobotTaskReference(x_dot_ref)

            # Draw trail line of the end-effector path while debugging#231996
            if debug and self.hasPrevPose:
                p.addUserDebugLine(self.prevPose, self.x, [1, 0, 0], 1, self.trailDuration)

            # Keep track of previous iteration's position.
            self.prevPose = self.x
            self.hasPrevPose = True

    def experimentResults(self):
        
        traj = open("trajectory.obs" , "wb")
        pickle.dump(self.trajectory, traj)
        traj.close()

        qtraj = open("joint_trajectories.noobs", "wb")
        pickle.dump(self.q_list, qtraj)
        qtraj.close()

        print("Reached target in: ",self.t," seconds")
        time.sleep(10)
        p.disconnect()

# Runs the following code when run from command line
if __name__ == "__main__":
    debug = "debug" in sys.argv

    # This is the line to be modified to get different results
    KukaRobotExperiment(debug = debug)

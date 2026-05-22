from simulator.robot import Robot
import numpy as np
import pybullet as pb
import pinocchio as pin

from simulator.body import Body
import simulator.utilities as sim_utils
class Teleoperation(Robot):
     urdf = "src/talos_description/robots/talos_reduced.urdf"
     path_meshes = "src/talos_description/meshes/../.."

     self.wrapper = pin.RobotWrapper.BuildFromURDF(urdf,                        # Model description
                                           path_meshes,                 # Model geometry descriptors 
                                           None,   # Floating base model. Use "None" if fixed
                                           True,                        # Printout model details
                                           None)                        # Load meshes different from the descripor
     # Get model from wrapper
     self.model= wrapper.model
     def __init__(self, 
                 simulator,
                 node):

        super().__init__(
                 simulator=simulator,
                 filename=filename,
                 model=self.model,
                 use_fixed_base=True,
                 
                 )

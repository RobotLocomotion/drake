import os
import numpy as np
import pydrake
from pydrake.solvers import ik

robot = pydrake.rbtree.RigidBodyTree(os.path.join(pydrake.getDrakePath(), "examples/PR2/pr2.urdf"))

# Make sure attribute access works on bodies
assert robot.bodies[0].linkname == "world"

constraints = [ik.WorldPositionConstraint(robot, 1,
                                          np.zeros((3,)),
                                          np.array([0,0,1.0]),
                                          np.array([0,0,1.0])),
               ik.WorldPositionConstraint(robot, robot.findLink("r_gripper_palm_link").body_index,
                                          np.zeros((3,)),
                                          np.array([0.5,0,1.5]),
                                          np.array([0.5,0,1.5])),
               ]

q_seed = robot.getZeroConfiguration()
options = ik.IKoptions(robot)
results = ik.inverseKinSimple(robot, q_seed, q_seed, constraints, options)
print repr(results.q_sol)

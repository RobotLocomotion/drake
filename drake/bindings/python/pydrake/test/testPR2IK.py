from __future__ import print_function

import os
import numpy as np
import pydrake
from pydrake.solvers import ik


def load_robot_from_urdf(urdf_file):
    """
    This function demonstrates how to pass a complete
    set of arguments to Drake's URDF parser.  It is also
    possible to load a robot with a much simpler syntax
    that uses default values, such as:

      robot = pydrake.rbtree.RigidBodyTree(urdf_file)

    """
    urdf_string = open(urdf_file).read()
    base_dir = os.path.dirname(urdf_file)
    package_map = pydrake.rbtree.PackageMap()
    weld_frame = None
    floating_base_type = pydrake.rbtree.kRollPitchYaw

    # Load our model from URDF
    robot = pydrake.rbtree.RigidBodyTree()

    pydrake.rbtree.AddModelInstanceFromUrdfStringSearchingInRosPackages(
      urdf_string,
      package_map,
      base_dir,
      floating_base_type,
      weld_frame,
      robot)

    return robot

urdf_file = os.path.join(pydrake.getDrakePath(), "examples/PR2/pr2.urdf")

# Load our model from URDF
robot = load_robot_from_urdf(urdf_file)

# Add a convenient frame, positioned 0.1m away from the r_gripper_palm_link
# along that link's x axis
robot.addFrame(
           pydrake.rbtree.RigidBodyFrame("r_hand_frame",
                                         robot.FindBody("r_gripper_palm_link"),
                                         np.array([0.1, 0, 0]),
                                         np.array([0., 0, 0])))

# Make sure attribute access works on bodies
assert robot.world().get_name() == "world"

hand_frame_id = robot.findFrame("r_hand_frame").get_frame_index()
base_body_id = robot.FindBody('base_footprint').get_body_index()

constraints = [
               # These three constraints ensure that the base of the robot is
               # at z = 0 and has no pitch or roll. Instead of directly
               # constraining orientation, we just require that the points at
               # [0, 0, 0], [1, 0, 0], and [0, 1, 0] in the robot's base's
               # frame must all be at z = 0 in world frame.
               # We don't care about the x or y position of the robot's base,
               # so we use NaN values to tell the IK solver not to apply a
               # constraint along those dimensions. This is equivalent to
               # placing a lower bound of -Inf and an upper bound of +Inf along
               # those axes.
               ik.WorldPositionConstraint(robot, base_body_id,
                                          np.array([0.0, 0.0, 0.0])[:, None],
                                          np.array([np.nan, np.nan, 0.0]),
                                          np.array([np.nan, np.nan, 0.0])),
               ik.WorldPositionConstraint(robot, base_body_id,
                                          np.array([1.0, 0.0, 0.0])[:, None],
                                          np.array([np.nan, np.nan, 0.0]),
                                          np.array([np.nan, np.nan, 0.0])),
               ik.WorldPositionConstraint(robot, base_body_id,
                                          np.array([0.0, 1.0, 0.0])[:, None],
                                          np.array([np.nan, np.nan, 0.0]),
                                          np.array([np.nan, np.nan, 0.0])),

               # This constraint exactly specifies the desired position of the
               # hand frame we defined earlier.
               ik.WorldPositionConstraint(robot, hand_frame_id,
                                          np.array([0.0, 0.0, 0.0])[:, None],
                                          np.array([0.5, 0.0, 0.6]),
                                          np.array([0.5, 0.0, 0.6])),
               # And this specifies the orientation of that frame
               ik.WorldEulerConstraint(robot, hand_frame_id,
                                       np.array([0.0, 0.0, 0.0]),
                                       np.array([0.0, 0.0, 0.0]))
               ]

q_seed = robot.getZeroConfiguration()
options = ik.IKoptions(robot)
results = ik.InverseKin(robot, q_seed, q_seed, constraints, options)

# Each entry (only one is present in this case, since InverseKin()
# only returns a single result) in results.info gives the output
# status of SNOPT. info = 1 is good, anything less than 10 is OK, and
# any info >= 10 indicates an infeasibility or failure of the
# optimizer.
assert results.info[0] == 1

print(repr(results.q_sol[0]))

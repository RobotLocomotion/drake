import numpy as np

from pydrake.common.eigen_geometry import AngleAxis
from pydrake.manipulation.planner import DoDifferentialInverseKinematics
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.systems.framework import BasicVector, LeafSystem, PortDataType


# TODO(russt): Clean this up and move it to C++.
class DifferentialIK(LeafSystem):
    """
    A simple system that wraps calls to the DifferentialInverseKinematics API.
    It is highly recommended that the user calls SetPosition() once to
    initialize the initial position commands to match the initial
    configuration of the robot.

    .. pydrake_system::

        name: DifferentialIK
        input_ports:
        - X_WE_desired
        output_ports:
        - joint_position_desired
    """
    def __init__(self, robot, frame_E, parameters, time_step):
        """
        @param robot is a reference to a MultibodyPlant.
        @param frame_E is a multibody::Frame on the robot.
        @param params is a DifferentialIKParams.
        @params time_step This system updates its state/outputs at discrete
                          periodic intervals defined with period @p time_step.
        """
        LeafSystem.__init__(self)
        self.robot = robot
        self.frame_E = frame_E
        self.parameters = parameters
        self.parameters.set_timestep(time_step)
        self.time_step = time_step
        # Note that this context is NOT the context of the DifferentialIK
        # system, but rather a context for the multibody plant that is used
        # to pass the configuration into the DifferentialInverseKinematics
        # methods.
        self.robot_context = robot.CreateDefaultContext()
        # Confirm that all velocities are zero (they will not be reset below).
        assert not self.robot.GetPositionsAndVelocities(
            self.robot_context)[-robot.num_velocities():].any()

        # Store the robot positions as state.
        position_state_index = self.DeclareDiscreteState(robot.num_positions())
        self.DeclarePeriodicDiscreteUpdate(time_step)

        # Desired pose of frame E in world frame.
        self.DeclareInputPort("rpy_xyz_desired",
                              PortDataType.kVectorValued, 6)

        # Provide the output as desired positions.
        self.DeclareStateOutputPort("joint_position_desired",
                                    position_state_index)

    def SetPositions(self, context, q):
        context.SetDiscreteState(0, q)

    def ForwardKinematics(self, q):
        x = self.robot.GetMutablePositionsAndVelocities(
            self.robot_context)
        x[:self.robot.num_positions()] = q
        return self.robot.EvalBodyPoseInWorld(
            self.robot_context, self.frame_E.body())

    def CalcPoseError(self, X_WE_desired, q):
        pose = self.ForwardKinematics(q)
        err_vec = np.zeros(6)
        err_vec[-3:] = X_WE_desired.translation() - pose.translation()

        rot_err = AngleAxis(X_WE_desired.rotation()
                            * pose.rotation().transpose())
        err_vec[:3] = rot_err.axis() * rot_err.angle()

    def DoCalcDiscreteVariableUpdates(
            self, context, events, discrete_state):
        rpy_xyz_desired = self.EvalVectorInput(context, 0).get_value()
        X_WE_desired = RigidTransform(RollPitchYaw(rpy_xyz_desired[:3]),
                                      rpy_xyz_desired[-3:])
        q_last = context.get_discrete_state_vector().get_value()

        x = self.robot.GetMutablePositionsAndVelocities(
            self.robot_context)
        x[:self.robot.num_positions()] = q_last
        result = DoDifferentialInverseKinematics(self.robot,
                                                 self.robot_context,
                                                 X_WE_desired, self.frame_E,
                                                 self.parameters)

        if (result.status != result.status.kSolutionFound):
            print("Differential IK could not find a solution.")
            discrete_state.set_value(q_last)
        else:
            discrete_state.set_value(
                q_last + self.time_step * result.joint_velocities)

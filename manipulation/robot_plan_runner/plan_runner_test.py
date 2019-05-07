import numpy as np
import argparse

from pydrake.examples.manipulation_station import ManipulationStation
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.manipulation.robot_plan_runner import (
    PlanData,
    PlanSender,
    PlanType,
    RobotPlanRunner
)
from pydrake.multibody import inverse_kinematics
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.trajectories import PiecewisePolynomial


def GetEndEffectorWorldAlignedFrame():
    X_EEa = RigidTransform.Identity()
    X_EEa.set_rotation(RotationMatrix(np.array([[0., 1., 0,],
                                 [0, 0, 1],
                                 [1, 0, 0]])))
    return X_EEa

def GetKukaQKnots(plant, iiwa_model, q_knots):
    if len(q_knots.shape) == 1:
        q_knots.resize(1, q_knots.size)
    n = q_knots.shape[0]
    q_knots_kuka = np.zeros((n, 7))
    for i, q_knot in enumerate(q_knots):
        q_knots_kuka[i] = plant.GetPositionsFromArray(iiwa_model, q_knot)

    return q_knots_kuka

def GoFromPointToPoint(plant,
                       iiwa_model,
                       world_frame,
                       gripper_frame,
                       p_WQ_start,
                       p_WQ_end,
                       duration,
                       num_knot_points,
                       q_initial_guess):
    # The first knot point is the zero posture.
    # The second knot is the pre-pre-grasp posture q_val_0
    # The rest are solved for in the for loop below.
    # The hope is that using more knot points makes the trajectory
    # smoother.
    q_knots = np.zeros((num_knot_points+1, plant.num_positions()))
    q_knots[0] = q_initial_guess

    X_EEa = GetEndEffectorWorldAlignedFrame()

    for i in range(num_knot_points):
        ik = inverse_kinematics.InverseKinematics(plant)
        q_variables = ik.q()

        ik.AddOrientationConstraint(
            frameAbar=world_frame, R_AbarA=RollPitchYaw(0, np.pi*3/4, 0).ToRotationMatrix(),
            frameBbar=gripper_frame, R_BbarB=RotationMatrix(X_EEa.rotation()),
            theta_bound=0.01 * np.pi)

        p_WQ = (p_WQ_end - p_WQ_start)/num_knot_points*(i+1) + p_WQ_start

        ik.AddPositionConstraint(
            frameB=gripper_frame, p_BQ=np.array([0., 0.12, 0.]),
            frameA=world_frame,
            p_AQ_lower=p_WQ-0.005, p_AQ_upper=p_WQ+0.005)

        prog = ik.prog()
        # use the robot posture at the previous knot point as
        # an initial guess.
        prog.SetInitialGuess(q_variables, q_knots[i])
        result = prog.Solve()
        print i, ": ", result
        q_knots[i+1] = prog.GetSolution(q_variables)

    t_knots = np.linspace(0, duration, num_knot_points + 1)

    q_knots_kuka = GetKukaQKnots(plant, iiwa_model, q_knots)
    qtraj = PiecewisePolynomial.Cubic(
        t_knots, q_knots_kuka.T,
        np.zeros(7), np.zeros(7))

    return qtraj, q_knots

def GeneratePlans(station):
    # get first pre-pre-grasp pose
    plant = station.get_mutable_multibody_plant()
    iiwa_model = plant.GetModelInstanceByName("iiwa")
    gripper_model = plant.GetModelInstanceByName("gripper")

    world_frame = plant.world_frame()
    gripper_frame = plant.GetFrameByName("body", gripper_model)

    theta_bound = 0.01 * np.pi
    X_EEa = GetEndEffectorWorldAlignedFrame()
    R_WEa = RollPitchYaw(0, np.pi*3/4, 0).ToRotationMatrix()
    R_EEa = RotationMatrix(X_EEa.rotation())

    ik_scene = inverse_kinematics.InverseKinematics(plant)
    prog = ik_scene.prog()
    prog.SetInitialGuess(ik_scene.q(), np.zeros(plant.num_positions()))
    result = prog.Solve()
    print result
    q_val_0 = prog.GetSolution(ik_scene.q())

    p_WQ_start = np.array([0.5, 0, 0.41])
    p_WQ_end = np.array([0.6, 0.1, 0.41])
    num_knot_points = 15
    qtraj, qknots = GoFromPointToPoint(plant,
                                       iiwa_model,
                                       world_frame,
                                       gripper_frame,
                                       p_WQ_start,
                                       p_WQ_end,
                                       5.,
                                       num_knot_points,
                                       q_val_0)

    plan_go_home = PlanData(PlanType.kJointSpacePlan, 1, qtraj)

    return [plan_go_home]

def RenderSystemWithGraphviz(system, output_file="system_view.gz"):
    """ Renders the Drake system (presumably a diagram,
    otherwise this graph will be fairly trivial) using
    graphviz to a specified file. """
    from graphviz import Source
    string = system.GetGraphvizString()
    src = Source(string)
    src.render(output_file, view=False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--duration", type=float, default=0.1,
        help="Desired duration of the simulation in seconds.")

    MeshcatVisualizer.add_argparse_argument(parser)
    args = parser.parse_args()

    builder = DiagramBuilder()

    # Create the Manipulation Station
    station = builder.AddSystem(ManipulationStation())
    station.SetupDefaultStation()
    station.Finalize()

    # Create the PlanSender
    plans = GeneratePlans(station)
    plan_sender = builder.AddSystem(PlanSender(plans))

    # Create the RobotPlanRunner
    plan_runner = builder.AddSystem(RobotPlanRunner())

    # Create MeshcatVisualizer
    meshcat = builder.AddSystem(MeshcatVisualizer(
        station.get_scene_graph(), zmq_url=args.meshcat,
        open_browser=args.open_browser))

    # Connect ports
    builder.Connect(station.GetOutputPort("pose_bundle"),
                    meshcat.get_input_port(0))

    builder.Connect(station.GetOutputPort("iiwa_position_measured"),
                    plan_runner.GetInputPort("iiwa_position_measured"))
    builder.Connect(station.GetOutputPort("iiwa_velocity_estimated"),
                    plan_runner.GetInputPort("iiwa_velocity_estimated"))
    builder.Connect(station.GetOutputPort("iiwa_torque_external"),
                    plan_runner.GetInputPort("iiwa_torque_external"))

    builder.Connect(station.GetOutputPort("iiwa_position_measured"),
                    plan_sender.GetInputPort("q"))

    builder.Connect(plan_sender.GetOutputPort("plan_data"),
                    plan_runner.GetInputPort("plan_data"))

    builder.Connect(plan_runner.GetOutputPort("iiwa_position_command"),
                    station.GetInputPort("iiwa_position"))
    builder.Connect(plan_runner.GetOutputPort("iiwa_torque_command"),
                    station.GetInputPort("iiwa_feedforward_torque"))

    # Build and simulate
    diagram = builder.Build()
    simulator = Simulator(diagram)

    station_context = diagram.GetMutableSubsystemContext(
        station, simulator.get_mutable_context())

    station_context.FixInputPort(
        station.GetInputPort("wsg_position").get_index(), [0.05])
    station_context.FixInputPort(
        station.GetInputPort("wsg_force_limit").get_index(), [50])

    RenderSystemWithGraphviz(diagram)

    q0 = np.array([[0, 0, 0, -1.75, 0, 1.0, 0]]).T
    station.SetIiwaPosition(station_context, q0)

    simulator.set_publish_every_time_step(False)
    simulator.set_target_realtime_rate(1.0) # go as fast as possible

    simulator.Initialize()
    simulator.AdvanceTo(plan_sender.get_all_plans_duration())
from pydrake.automotive import (
    DrivingCommand,
    IdmController,
    LaneDirection,
    PurePursuitController,
    RoadPositionStrategy,
    ScanStrategy,
    SimpleCar,
    SimpleCarState,
)

import unittest

import numpy as np

import pydrake.systems.framework as framework
from pydrake.maliput.api import (
    LanePosition,
    RoadGeometryId,
)
from pydrake.maliput.dragway import (
    create_dragway,
)
from pydrake.multibody.multibody_tree.math import (
    SpatialVelocity,
)
from pydrake.systems.analysis import (
    Simulator,
)
from pydrake.systems.rendering import (
    FrameVelocity,
    PoseBundle,
    PoseVector,
)
from pydrake.util.eigen_geometry import (
    Isometry3,
    Quaternion,
)


def make_two_lane_road():
    return create_dragway(
        road_id=RoadGeometryId("two_lanes"), num_lanes=2, length=10.,
        lane_width=4., shoulder_width=0., maximum_height=1.,
        linear_tolerance=1e-6, angular_tolerance=1e-6)


class TestAutomotive(unittest.TestCase):
    def test_lane_direction(self):
        rg = make_two_lane_road()
        lane1 = rg.junction(0).segment(0).lane(0)
        lane_direction = LaneDirection(lane=lane1, with_s=True)
        test_coordinate = LanePosition(s=1., r=2., h=3.)
        self.assertTrue(np.allclose(
            lane_direction.lane.ToGeoPosition(test_coordinate).xyz(),
            lane1.ToGeoPosition(test_coordinate).xyz()))
        self.assertEqual(lane_direction.with_s, True)
        lane2 = rg.junction(0).segment(0).lane(1)
        lane_direction.lane = lane2
        lane_direction.with_s = False
        self.assertTrue(np.allclose(
            lane_direction.lane.ToGeoPosition(test_coordinate).xyz(),
            lane2.ToGeoPosition(test_coordinate).xyz()))
        self.assertEqual(lane_direction.with_s, False)

    def test_driving_command(self):
        driving_command = DrivingCommand()
        self.assertTrue(isinstance(driving_command, framework.BasicVector))
        self.assertEqual(driving_command.size(), 2)
        self.assertNotEqual(driving_command.steering_angle(), 5.)
        driving_command.set_steering_angle(5.)
        self.assertEqual(driving_command.steering_angle(), 5.)
        self.assertNotEqual(driving_command.acceleration(), 23.)
        driving_command.set_acceleration(23.)
        self.assertEqual(driving_command.acceleration(), 23.)

    def test_pure_pursuit(self):
        rg = make_two_lane_road()
        lane = rg.junction(0).segment(0).lane(0)
        pure_pursuit = PurePursuitController()
        context = pure_pursuit.CreateDefaultContext()
        output = pure_pursuit.AllocateOutput(context)

        # Fix the inputs.
        ld_value = framework.AbstractValue.Make(
            LaneDirection(lane=lane, with_s=True))
        lane_index = pure_pursuit.lane_input().get_index()
        context.FixInputPort(lane_index, ld_value)

        pos = [1., 2., 3.]  # An aribtrary position with the lane.
        pose_vector = PoseVector()
        pose_vector.set_translation(pos)
        pose_index = pure_pursuit.ego_pose_input().get_index()
        context.FixInputPort(pose_index, pose_vector)

        # Verify the inputs.
        pose_vector_eval = pure_pursuit.EvalVectorInput(context, pose_index)
        self.assertTrue(np.allclose(pose_vector.get_translation(),
                                    pose_vector_eval.get_translation()))

        # Verify the outputs.
        pure_pursuit.CalcOutput(context, output)
        command_index = pure_pursuit.steering_command_output().get_index()
        steering = output.get_vector_data(command_index)
        self.assertEqual(len(steering.get_value()), 1)
        self.assertTrue(steering.get_value() < 0.)

    def test_simple_car_state(self):
        simple_car_state = SimpleCarState()
        self.assertTrue(isinstance(simple_car_state, framework.BasicVector))
        self.assertEqual(simple_car_state.size(), 4)
        self.assertNotEqual(simple_car_state.x(), 5.)
        simple_car_state.set_x(5.)
        self.assertEqual(simple_car_state.x(), 5.)
        self.assertNotEqual(simple_car_state.y(), 2.)
        simple_car_state.set_y(2.)
        self.assertEqual(simple_car_state.y(), 2.)
        self.assertNotEqual(simple_car_state.heading(), 14.)
        simple_car_state.set_heading(14.)
        self.assertEqual(simple_car_state.heading(), 14.)
        self.assertNotEqual(simple_car_state.velocity(), 52.)
        simple_car_state.set_velocity(52.)
        self.assertEqual(simple_car_state.velocity(), 52.)

    def test_idm_controller(self):
        rg = make_two_lane_road()
        idm = IdmController(
            road=rg, path_or_branches=ScanStrategy.kPath,
            road_position_strategy=RoadPositionStrategy.kExhaustiveSearch,
            period_sec=0.)
        context = idm.CreateDefaultContext()
        output = idm.AllocateOutput(context)

        # Fix the inputs.
        pose_vector1 = PoseVector()
        pose_vector1.set_translation([1., 2., 3.])
        ego_pose_index = idm.ego_pose_input().get_index()
        context.FixInputPort(ego_pose_index, pose_vector1)

        w = [0., 0., 0.]
        v = [1., 0., 0.]
        frame_velocity1 = FrameVelocity(SpatialVelocity(w=w, v=v))
        ego_velocity_index = idm.ego_velocity_input().get_index()
        context.FixInputPort(ego_velocity_index, frame_velocity1)

        pose_vector2 = PoseVector()
        pose_vector2.set_translation([6., 0., 0.])
        bundle = PoseBundle(num_poses=1)
        bundle.set_pose(
            0, Isometry3(Quaternion(), pose_vector2.get_translation()))
        traffic_index = idm.traffic_input().get_index()
        context.FixInputPort(
            traffic_index, framework.AbstractValue.Make(bundle))

        # Verify the inputs.
        pose_vector_eval = idm.EvalVectorInput(context, 0)
        self.assertTrue(np.allclose(pose_vector1.get_translation(),
                                    pose_vector_eval.get_translation()))
        frame_velocity_eval = idm.EvalVectorInput(context, 1)
        self.assertTrue(np.allclose(
            frame_velocity1.get_velocity().translational(),
            frame_velocity_eval.get_velocity().translational()))
        self.assertTrue(np.allclose(
            frame_velocity1.get_velocity().rotational(),
            frame_velocity_eval.get_velocity().rotational()))

        # Verify the outputs.
        idm.CalcOutput(context, output)
        accel_command_index = idm.acceleration_output().get_index()
        accel = output.get_vector_data(accel_command_index)
        self.assertEqual(len(accel.get_value()), 1)
        self.assertTrue(accel.get_value() < 0.)

    def test_simple_car(self):
        simple_car = SimpleCar()
        simulator = Simulator(simple_car)
        context = simulator.get_mutable_context()
        output = simple_car.AllocateOutput(context)

        # Fix the input.
        command = DrivingCommand()
        command.set_steering_angle(0.5)
        command.set_acceleration(1.)
        context.FixInputPort(0, command)

        # Verify the inputs.
        command_eval = simple_car.EvalVectorInput(context, 0)
        self.assertTrue(np.allclose(
            command.get_value(), command_eval.get_value()))

        # Initialize all the states to zero and take a simulation step.
        state = context.get_mutable_continuous_state_vector()
        state.SetFromVector([0.] * state.size())
        simulator.StepTo(1.0)

        # Verify the outputs.
        simple_car.CalcOutput(context, output)
        state_index = simple_car.state_output().get_index()
        state_value = output.get_vector_data(state_index)
        self.assertIsInstance(state_value, SimpleCarState)
        self.assertTrue(
            np.allclose(state.CopyToVector(), state_value.get_value()))
        pose_index = simple_car.pose_output().get_index()
        pose_value = output.get_vector_data(pose_index)
        self.assertIsInstance(pose_value, PoseVector)
        self.assertTrue(pose_value.get_translation()[0] > 0.)
        velocity_index = simple_car.velocity_output().get_index()
        velocity_value = output.get_vector_data(velocity_index)
        self.assertIsInstance(velocity_value, FrameVelocity)
        self.assertTrue(velocity_value.get_velocity().translational()[0] > 0.)

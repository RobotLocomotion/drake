from pydrake.automotive import (
    AheadOrBehind,
    ClosestPose,
    DrivingCommand,
    IdmController,
    LaneDirection,
    PoseSelector,
    PurePursuitController,
    RoadOdometry,
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
    RoadPosition,
)
from pydrake.maliput.dragway import (
    create_dragway,
)
from pydrake.multibody.math import (
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
from pydrake.common.eigen_geometry import (
    Isometry3,
    Quaternion,
)


def make_two_lane_road():
    return create_dragway(
        road_id=RoadGeometryId("two_lanes"), num_lanes=2, length=10.,
        lane_width=4., shoulder_width=0., maximum_height=1.,
        linear_tolerance=1e-6, angular_tolerance=1e-6)


class TestAutomotive(unittest.TestCase):
    def test_road_odometry(self):
        RoadOdometry()
        rg = make_two_lane_road()
        lane_0 = rg.junction(0).segment(0).lane(0)
        lane_1 = rg.junction(0).segment(0).lane(1)
        lane_pos = LanePosition(s=1., r=2., h=3.)
        road_pos = RoadPosition(lane=lane_0, pos=lane_pos)
        w = [5., 7., 9.]
        v = [11., 13., 15.]
        frame_velocity = FrameVelocity(SpatialVelocity(w=w, v=v))
        road_odom = RoadOdometry(
            road_position=road_pos, frame_velocity=frame_velocity)

        self.assertEqual(road_odom.lane.id().string(), lane_0.id().string())
        self.assertTrue(np.allclose(road_odom.pos.srh(), lane_pos.srh()))
        self.assertTrue(np.allclose(
            frame_velocity.get_velocity().translational(),
            road_odom.vel.get_velocity().translational()))
        lane_pos_new = LanePosition(s=10., r=20., h=30.)
        v_new = [42., 43., 44.]
        frame_velocity_new = FrameVelocity(SpatialVelocity(w=w, v=v_new))
        road_odom.lane = lane_1
        road_odom.pos = lane_pos_new
        road_odom.vel = frame_velocity_new

        self.assertEqual(road_odom.lane.id().string(), lane_1.id().string())
        self.assertTrue(np.allclose(road_odom.pos.srh(), lane_pos_new.srh()))
        self.assertTrue(np.allclose(
            frame_velocity_new.get_velocity().translational(),
            road_odom.vel.get_velocity().translational()))

    def test_closest_pose(self):
        ClosestPose()
        rg = make_two_lane_road()
        lane_0 = rg.junction(0).segment(0).lane(0)
        lane_1 = rg.junction(0).segment(0).lane(1)
        lane_pos = LanePosition(s=1., r=2., h=3.)
        road_pos = RoadPosition(lane=lane_0, pos=lane_pos)
        w = [5., 7., 9.]
        v = [11., 13., 15.]
        frame_velocity = FrameVelocity(SpatialVelocity(w=w, v=v))
        road_odom = RoadOdometry(
            road_position=road_pos, frame_velocity=frame_velocity)

        closest_pose = ClosestPose(odom=road_odom, dist=12.7)
        self.assertEqual(closest_pose.odometry.lane.id().string(),
                         lane_0.id().string())
        self.assertEqual(closest_pose.distance, 12.7)

        closest_pose.odometry.lane = lane_1
        closest_pose.distance = 5.9
        self.assertEqual(closest_pose.odometry.lane.id().string(),
                         lane_1.id().string())
        self.assertEqual(closest_pose.distance, 5.9)

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
        output = pure_pursuit.AllocateOutput()

        # Fix the inputs.
        ld_value = framework.AbstractValue.Make(
            LaneDirection(lane=lane, with_s=True))
        lane_index = pure_pursuit.lane_input().get_index()
        context.FixInputPort(lane_index, ld_value)

        pos = [1., 2., 3.]  # An arbitrary position with the lane.
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
        output = idm.AllocateOutput()

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
        output = simple_car.AllocateOutput()

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
        simulator.AdvanceTo(1.0)

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

    def test_pose_selector(self):
        kScanDistance = 4.
        rg = make_two_lane_road()
        lane = rg.junction(0).segment(0).lane(0)
        pose0 = PoseVector()
        pose0.set_translation([1., 0., 0.])
        pose1 = PoseVector()
        # N.B. Set pose1 3 meters ahead of pose0.
        pose1.set_translation([4., 0., 0.])

        bundle = PoseBundle(num_poses=2)
        bundle.set_pose(0, Isometry3(Quaternion(), pose0.get_translation()))
        bundle.set_pose(1, Isometry3(Quaternion(), pose1.get_translation()))

        closest_pose = PoseSelector.FindSingleClosestPose(
            lane=lane, ego_pose=pose0, traffic_poses=bundle,
            scan_distance=kScanDistance, side=AheadOrBehind.kAhead,
            path_or_branches=ScanStrategy.kPath)
        self.assertEqual(closest_pose.odometry.lane.id().string(),
                         lane.id().string())
        self.assertTrue(closest_pose.distance == 3.)

        closest_pair = PoseSelector.FindClosestPair(
            lane=lane, ego_pose=pose0, traffic_poses=bundle,
            scan_distance=kScanDistance, path_or_branches=ScanStrategy.kPath)
        self.assertEqual(
            closest_pair[AheadOrBehind.kAhead].odometry.lane.id().string(),
            lane.id().string())
        self.assertEqual(closest_pair[AheadOrBehind.kAhead].distance, 3.)
        self.assertEqual(
            closest_pair[AheadOrBehind.kBehind].odometry.lane.id().string(),
            lane.id().string())
        self.assertEqual(closest_pair[AheadOrBehind.kBehind].distance,
                         float('inf'))

        lane_pos = LanePosition(s=1., r=0., h=0.)
        road_pos = RoadPosition(lane=lane, pos=lane_pos)
        w = [1., 2., 3.]
        v = [-4., -5., -6.]
        frame_velocity = FrameVelocity(SpatialVelocity(w=w, v=v))
        road_odom = RoadOdometry(
            road_position=road_pos, frame_velocity=frame_velocity)
        sigma_v = PoseSelector.GetSigmaVelocity(road_odom)
        self.assertEqual(sigma_v, v[0])

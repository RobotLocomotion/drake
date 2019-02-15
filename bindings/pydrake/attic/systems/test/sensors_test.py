import pydrake.attic.systems.sensors as attic_mut
import pydrake.systems.sensors as mut

import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.eigen_geometry import Isometry3
from pydrake.attic.multibody.rigid_body_tree import (
    AddModelInstancesFromSdfString,
    FloatingBaseType,
    RigidBodyTree,
    RigidBodyFrame,
)
from pydrake.systems.framework import InputPort, OutputPort, Value


class TestSensors(unittest.TestCase):
    def _check_input(self, value):
        self.assertIsInstance(value, InputPort)

    def _check_output(self, value):
        self.assertIsInstance(value, OutputPort)

    def _check_ports(self, system):
        self._check_input(system.state_input_port())
        self._check_output(system.color_image_output_port())
        self._check_output(system.depth_image_output_port())
        self._check_output(system.label_image_output_port())
        self._check_output(system.camera_base_pose_output_port())

    def test_rgbd_camera(self):
        sdf_path = FindResourceOrThrow(
            "drake/systems/sensors/test/models/nothing.sdf")
        tree = RigidBodyTree()
        with open(sdf_path) as f:
            sdf_string = f.read()
        AddModelInstancesFromSdfString(
            sdf_string, FloatingBaseType.kFixed, None, tree)
        frame = RigidBodyFrame("rgbd camera frame", tree.FindBody("link"))
        tree.addFrame(frame)

        # Use HDTV size.
        width = 1280
        height = 720

        camera = attic_mut.RgbdCamera(
            name="camera", tree=tree, frame=frame,
            z_near=0.5, z_far=5.0,
            fov_y=np.pi / 4, show_window=False,
            width=width, height=height)

        def check_info(camera_info):
            self.assertIsInstance(camera_info, mut.CameraInfo)
            self.assertEqual(camera_info.width(), width)
            self.assertEqual(camera_info.height(), height)

        check_info(camera.color_camera_info())
        check_info(camera.depth_camera_info())
        self.assertIsInstance(camera.color_camera_optical_pose(), Isometry3)
        self.assertIsInstance(camera.depth_camera_optical_pose(), Isometry3)
        self.assertTrue(camera.tree() is tree)
        # N.B. `RgbdCamera` copies the input frame.
        self.assertEqual(camera.frame().get_name(), frame.get_name())
        self._check_ports(camera)

        # Test discrete camera.
        period = attic_mut.RgbdCameraDiscrete.kDefaultPeriod
        discrete = attic_mut.RgbdCameraDiscrete(
            camera=camera, period=period, render_label_image=True)
        self.assertTrue(discrete.camera() is camera)
        self.assertTrue(discrete.mutable_camera() is camera)
        self.assertEqual(discrete.period(), period)
        self._check_ports(discrete)

        # That we can access the state as images.
        context = discrete.CreateDefaultContext()
        values = context.get_abstract_state()
        self.assertIsInstance(values.get_value(0), Value[mut.ImageRgba8U])
        self.assertIsInstance(values.get_value(1), Value[mut.ImageDepth32F])
        self.assertIsInstance(values.get_value(2), Value[mut.ImageLabel16I])

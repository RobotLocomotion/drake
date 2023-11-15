import asyncio
import logging
import numpy as np
import unittest
import umsgpack
import websockets

import pydrake.common.test_utilities.numpy_compare as numpy_compare
from pydrake.geometry import Meshcat
from pydrake.math import (
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
)
import pydrake.visualization as mut

# BEGIN ugly hack
#
# https://bugs.launchpad.net/ubuntu/+source/python-websockets/+bug/1969902
#
# Python 3.10 (Jammy) removed the `loop` parameter from various asyncio APIs.
# Unfortunately, Jammy shipped with python3-websokets 9.1, which is still
# passing a `loop` parameter, resulting in asyncio expressing its displeasure
# by means of exceptions. Trying to patch websokets directly is not reasonable.
# Instead, patch the asyncio functions to remove the `loop` parameter.
#
# TODO(mwoehlke-kitware): Remove this when Jammy's python3-websockets has been
# updated to 10.0 or later.
#
# Note: this ugly hack is copied from meshcat_websocket_client.py.
_asyncio_lock_ctor = asyncio.Lock.__init__
_asyncio_wait_for = asyncio.wait_for
_asyncio_sleep = asyncio.sleep


def _patch_asyncio(orig):
    def _patched(*args, **kwargs):
        if 'loop' in kwargs and kwargs['loop'] is None:
            kwargs.pop('loop')
        return orig(*args, **kwargs)

    return _patched


asyncio.Lock.__init__ = _patch_asyncio(_asyncio_lock_ctor)
asyncio.wait_for = _patch_asyncio(_asyncio_wait_for)
asyncio.sleep = _patch_asyncio(_asyncio_sleep)

# END ugly hack


async def test_tracked_camera_pose(tester):
    logger = logging.getLogger("model_visualizer_camera_tracking")

    meshcat = Meshcat()

    # A message with a valid transform (16 floats and is perspective is True).
    valid_pose_message = {
        "type": "camera_pose",
        "camera_pose": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1],
        "is_perspective": True
    }
    # Transform y-up to z-up, and from facing in the +z direction to the -z
    # direction (with concomitant flip of the y-axis).
    X_WB_expected = RigidTransform(
        R=RotationMatrix(RollPitchYaw(np.pi / 2, np.pi, np.pi)),
        p=[1.0, -3.0, 2.0])

    # Configure a persistent client.
    logger.info("Connecting...")
    async with websockets.connect(meshcat.ws_url(),
                                  close_timeout=1) as websocket:
        logger.info("... connected")
        # send valid meshcat pose; model_visualizer should make use of it.
        logger.info("Sending valid pose...")
        await websocket.send(umsgpack.packb(valid_pose_message))
        logger.info("... sent")

        # Instantiate the model visualizer, confirm initial state, and then
        # run a single loop.

        # N.B. We don't need perception geometry in the scene -- we'll rely on
        # the RgbdSensor unit tests to check that cameras work as advertised.
        # Our only goal here is to achieve statement-level code coverage of the
        # ModelVisualizer code when show_rgbd_sensor=True.
        model = """<?xml version="1.0"?>
            <sdf version="1.9">
            <model name="sample">
                <link name="base"/>
            </model>
            </sdf>
        """
        dut = mut.ModelVisualizer(show_rgbd_sensor=True, meshcat=meshcat)
        dut.parser().AddModelsFromString(model, "sdf")
        dut.Finalize()

        # Initial world pose of the camera should be the identity.
        camera_frame = dut._diagram.plant().GetFrameByName(
            "$rgbd_sensor_offset")
        X_WB = camera_frame.GetPoseInParentFrame(
            dut._diagram.plant().GetMyContextFromRoot(
                dut._context))
        tester.assertTrue(X_WB.IsExactlyIdentity())

        # When we run one loop, we should update the pose.
        dut.Run(loop_once=True)

        # We're assuming the parent frame *is* the world frame.
        X_WB = camera_frame.GetPoseInParentFrame(
            dut._diagram.plant().GetMyContextFromRoot(
                dut._context))

        numpy_compare.assert_allclose(X_WB.GetAsMatrix34(),
                                      X_WB_expected.GetAsMatrix34())

    # Disconnect client -- pose is no longer available.
    logger.info("Connection closed")


class TestModelVisualizerCameraTracking(unittest.TestCase):

    def test_camera_tracking(self):
        """
        Tests model_visualizer's most direct responsibility for rendering the
        "right" image. model_visualizer is responsible for updating its context
        based on the camera pose Meshcat reports. We'll simply confirm that
        meshcat has successfully done so.
        """
        # We need a persistent websocket connection so the pose doesn't
        # cleared on disconnect.
        logging.basicConfig(
            level=logging.INFO,
            format="[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s")
        asyncio.run(test_tracked_camera_pose(self))

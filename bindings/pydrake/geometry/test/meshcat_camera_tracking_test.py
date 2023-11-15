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


async def ReadValidTransform(meshcat):
    """Indefinitely attempts reading a camera pose from meshcat."""
    X_WC = None
    while X_WC is None:
        X_WC = meshcat.GetTrackedCameraPose()
        if X_WC is None:
            await asyncio.sleep(0.1)
    return X_WC


async def test_undefined_tracked_camera_pose_protocol(tester):
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s")
    logger = logging.getLogger("meshcat_camera_tracking")
    dut = Meshcat()

    # When no message has been received, no pose is available.
    tester.assertIsNone(dut.GetTrackedCameraPose())

    # A message with a valid transform (16 floats and is perspective is True).
    valid_pose_message = {
        "type": "camera_pose",
        "camera_pose": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1],
        "is_perspective": True
    }
    # Transform y-up to z-up, and from facing in the +z direction to the -z
    # direction (with concomitant flip of the y-axis).
    X_WC_expected = RigidTransform(
        R=RotationMatrix(RollPitchYaw(np.pi / 2, np.pi, np.pi)),
        p=[1.0, -3.0, 2.0])

    # The message sent when the camera has an orthographic projection.
    invalid_pose_message = {
        "type": "camera_pose",
        "camera_pose": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1],
        "is_perspective": False
    }
    # Configure a client
    logger.info("Connecting...")
    async with websockets.connect(dut.ws_url(), close_timeout=1) as websocket:
        logger.info("... connected")
        # send valid meshcat pose - pose is available.
        logger.info("Sending valid pose...")
        await websocket.send(umsgpack.packb(valid_pose_message))
        logger.info("... sent")

        logger.info("Reading transform from meshcat...")
        X_WC = await asyncio.wait_for(ReadValidTransform(dut), timeout=1)
        logger.info("...read")
        # A message with a pose produces a pose in Meshcat.
        tester.assertIsNotNone(X_WC)
        # The pose has been transformed.
        numpy_compare.assert_allclose(X_WC.GetAsMatrix34(),
                                      X_WC_expected.GetAsMatrix34())

        # Send invalid meshcat pose - pose is cleared.
        logger.info("Sending invalid pose...")
        await websocket.send(umsgpack.packb(invalid_pose_message))
        logger.info("... sent")
        # A brief delay to accommodate any thread scheduling issues.
        await asyncio.sleep(0.1)
        # It has now been cleared because the pose isn't perspective.
        tester.assertIsNone(dut.GetTrackedCameraPose())

        # Send valid pose again - get back to a non-nullopt transform.
        logger.info("Resending valid pose...")
        await websocket.send(umsgpack.packb(valid_pose_message))
        logger.info("... sent")
        X_WC = await ReadValidTransform(dut)
        tester.assertIsNotNone(X_WC)

    # Disconnect client -- pose is no longer available.
    logger.info("Connection closed")
    await asyncio.sleep(0.1)
    tester.assertIsNone(dut.GetTrackedCameraPose())


class TestMeshcatCameraTracking(unittest.TestCase):
    """
    Tests the documented protocols for when Meshcat::GetTrackedCameraPose()
    returns nullopt vs a value.

    We can't use meshcat_websocket_client, because that utility creates a
    short term websocket connection to do a single task. The tracked pose
    logic depends on connection status. So, to fully test it, we need a
    persistent connection (to differentiate between nullopt due to message
    data vs nullopt due to lack of connection).

    This also gives a regression test on the transformation of the
    camera pose reported by meshcat. The three.js camera matrix is expressed
    in a y-up world frame. Drake's world is z up. In addition, the three.js's
    mapping from camera frame to image differs from Drake. This test confirms
    that reported pose is transformed as expected.
    """

    def test_camera_pose_message(self):
        asyncio.run(test_undefined_tracked_camera_pose_protocol(self))

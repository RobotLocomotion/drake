import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.attic.multibody.rigid_body_tree import (
    AddModelInstanceFromUrdfFile,
    FloatingBaseType,
    RigidBodyTree,
    RigidBodyFrame,
    )
from pydrake.systems.framework import BasicVector
from pydrake.systems.sensors import CameraInfo
from pydrake.attic.systems.sensors import RgbdCamera

# Create tree describing scene.
urdf_path = FindResourceOrThrow(
    "drake/multibody/models/box.urdf")
tree = RigidBodyTree()
AddModelInstanceFromUrdfFile(
    urdf_path, FloatingBaseType.kFixed, None, tree)
# - Add frame for camera fixture.
frame = RigidBodyFrame(
    name="rgbd camera frame",
    body=tree.world(),
    xyz=[-2, 0, 2],  # Ensure that the box is within range.
    rpy=[0, np.pi / 4, 0])
tree.addFrame(frame)

# Create camera.
camera = RgbdCamera(
    name="camera", tree=tree, frame=frame,
    z_near=0.5, z_far=5.0,
    fov_y=np.pi / 4, show_window=True)

# - Describe state.
x = np.zeros(tree.get_num_positions() + tree.get_num_velocities())

# Allocate context and render.
context = camera.CreateDefaultContext()
context.FixInputPort(0, BasicVector(x))
output = camera.AllocateOutput()
camera.CalcOutput(context, output)

# Get images from computed output.
color_index = camera.color_image_output_port().get_index()
color_image = output.get_data(color_index).get_value()
color_array = color_image.data

depth_index = camera.depth_image_output_port().get_index()
depth_image = output.get_data(depth_index).get_value()
depth_array = depth_image.data

# Show camera info and images.
print("Intrinsics:\n{}".format(camera.depth_camera_info().intrinsic_matrix()))
dpi = mpl.rcParams['figure.dpi']
figsize = np.array([color_image.width(), color_image.height()*2]) / dpi
plt.figure(1, figsize=figsize)
plt.subplot(2, 1, 1)
plt.imshow(color_array)
plt.subplot(2, 1, 2)
# mpl does not like singleton dimensions for single-channel images.
plt.imshow(np.squeeze(depth_array))
plt.show()

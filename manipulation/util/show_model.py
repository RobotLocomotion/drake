"""Loads a model file (*.sdf or *.urdf) and then commands drake-visualizer to
display it.
"""

import argparse
import numpy as np
import os
import sys

from pydrake.lcm import DrakeLcm
from pydrake.multibody.rigid_body_plant import DrakeVisualizer
from pydrake.multibody.rigid_body_tree import (
    AddModelInstancesFromSdfFile,
    AddModelInstanceFromUrdfFile,
    FloatingBaseType,
    RigidBodyTree,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("filename")
    args = parser.parse_args()

    # Load the model file.
    tree = RigidBodyTree()
    ext = os.path.splitext(args.filename)[1]
    if ext == ".sdf":
        AddModelInstancesFromSdfFile(
            args.filename, FloatingBaseType.kFixed, None, tree)
    elif ext == ".urdf":
        AddModelInstanceFromUrdfFile(
            args.filename, FloatingBaseType.kFixed, None, tree)
    else:
        parser.error("Unknown extension " + ext)

    # Send drake-visualizer messages to load the model and then position it in
    # its default configuration.
    q = tree.getZeroConfiguration()
    v = np.zeros(tree.get_num_velocities())
    lcm = DrakeLcm()
    visualizer = DrakeVisualizer(tree=tree, lcm=lcm)
    visualizer.PublishLoadRobot()
    context = visualizer.CreateDefaultContext()
    context.FixInputPort(0, np.concatenate([q, v]))
    visualizer.Publish(context)


if __name__ == '__main__':
    sys.exit(main())

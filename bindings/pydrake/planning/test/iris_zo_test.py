import unittest

import pydrake.planning as mut
from pydrake.common import RandomGenerator, Parallelism
from pydrake.geometry.optimization import Hyperellipsoid, HPolyhedron
from pydrake.planning import (
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
    CollisionCheckerParams,
)

import numpy as np


class TestIrisZo(unittest.TestCase):
    def test_iris_zo(self):
        # Taken from iris_from_clique_cover_test.py
        cross_cspace_urdf = """
        <robot name="boxes">
          <link name="fixed">
            <collision name="top_left">
              <origin rpy="0 0 0" xyz="-1 1 0"/>
              <geometry><box size="1 1 1"/></geometry>
            </collision>
            <collision name="top_right">
              <origin rpy="0 0 0" xyz="1 1 0"/>
              <geometry><box size="1 1 1"/></geometry>
            </collision>
            <collision name="bottom_left">
              <origin rpy="0 0 0" xyz="-1 -1 0"/>
              <geometry><box size="1 1 1"/></geometry>
            </collision>
            <collision name="bottom_right">
              <origin rpy="0 0 0" xyz="1 -1 0"/>
              <geometry><box size="1 1 1"/></geometry>
            </collision>
          </link>
          <joint name="fixed_link_weld" type="fixed">
            <parent link="world"/>
            <child link="fixed"/>
          </joint>
          <link name="movable">
            <collision name="sphere">
              <geometry><sphere radius="0.1"/></geometry>
            </collision>
          </link>
          <link name="for_joint"/>
          <joint name="x" type="prismatic">
            <axis xyz="1 0 0"/>
            <limit lower="-2" upper="2"/>
            <parent link="world"/>
            <child link="for_joint"/>
          </joint>
          <joint name="y" type="prismatic">
            <axis xyz="0 1 0"/>
            <limit lower="-2" upper="2"/>
            <parent link="for_joint"/>
            <child link="movable"/>
          </joint>
        </robot>
"""
        params = dict(edge_step_size=0.1)
        builder = RobotDiagramBuilder()
        params["robot_model_instances"] = builder.parser().AddModelsFromString(
            cross_cspace_urdf, "urdf"
        )
        params["model"] = builder.Build()
        plant = params["model"].plant()
        checker = SceneGraphCollisionChecker(**params)
        options = mut.IrisZoOptions()
        seed_point = np.zeros((2,))
        starting_ellipsoid = Hyperellipsoid.MakeHypersphere(0.01, seed_point)
        domain = HPolyhedron.MakeBox(plant.GetPositionLowerLimits(),
                                     plant.GetPositionUpperLimits())
        region = mut.IrisZo(checker, starting_ellipsoid, domain, options)
        test_point = np.array([0.0, 0.5])
        self.assertTrue(region.PointInSet(test_point))

from pydrake.multibody import rational

import numpy as np
import unittest

import pydrake
from pydrake.common import FindResourceOrThrow
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import (
    MultibodyPlant, AddMultibodyPlantSceneGraph)
from pydrake.symbolic import Polynomial, RationalFunction, Variable, Expression
from pydrake.multibody.parsing import Parser
from pydrake.multibody.tree import JointIndex
import pydrake.symbolic as sym



class TestRationalForwardKinematics(unittest.TestCase):
    def setUp(self):
        unittest.TestCase.setUp(self)

        builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            builder, MultibodyPlant(time_step=0.01))
        Parser(self.plant).AddModels(FindResourceOrThrow(
            "drake/bindings/pydrake/multibody/test/double_pendulum.sdf"))
        self.plant.Finalize()

        self.body0 = self.plant.GetBodyByName("base")
        self.body1 = self.plant.GetBodyByName("upper_link")
        self.body2 = self.plant.GetBodyByName("lower_link")

        diagram = builder.Build()

        # test constructor
        self.rat_forward = rational.RationalForwardKinematics(self. plant)

    def testGetters(self):
        plant = self.rat_forward.plant()
        s = self.rat_forward.s()

    def testCalcBodyPoseAsMultilinearPolynomial(self):
        # this also tests that rational::Pose is properly bound.
        q_star = np.zeros(self.plant.num_positions())

        pose = self.rat_forward.CalcBodyPoseAsMultilinearPolynomial(q_star=q_star,
                                                                    body_index=self.body2.index(),
                                                                    expressed_body_index=self.body0.index())
        self.assertEqual(pose.position().shape, (3,))
        self.assertEqual(pose.rotation().shape, (3,3))

        self.assertIsInstance(pose.position()[0], Polynomial)

    def testConvertMultilinearPolynomialToRationalFunction(self):
        q_star = np.zeros(self.plant.num_positions())

        pose = self.rat_forward.CalcBodyPoseAsMultilinearPolynomial(q_star=q_star,
                                                                    body_index=self.body2.index(),
                                                                    expressed_body_index=self.body0.index())
        multilinear_polynomial = pose.position()[0]
        rational_function = self.rat_forward.ConvertMultilinearPolynomialToRationalFunction(e=multilinear_polynomial)
        self.assertIsInstance(rational_function, RationalFunction)

    def testConversions(self):
        # Tests the double conversions
        q_star = np.zeros(self.plant.num_positions())
        q = np.ones(self.plant.num_positions())
        s = self.rat_forward.ComputeSValue(q_val=q, q_star_val=q_star)
        q_recomputed = self.rat_forward.ComputeQValue(s_val=s, q_star_val=q_star)
        self.assertTrue(np.allclose(q,  q_recomputed))

        # Tests the expression conversions
        x = Variable("x")
        q = np.array([Expression(x) for _ in range(self.plant.num_positions())])
        s = self.rat_forward.ComputeSValue(q_val=q, q_star_val=q_star)
        q_recomputed = self.rat_forward.ComputeQValue(s_val=s, q_star_val=q_star)
        self.assertTrue(all([q_recomputed_elt.EqualTo(sym.atan2(2*sym.tan(x/2), 1-(sym.tan(x/2))**2))
                             for q_recomputed_elt in q_recomputed]))

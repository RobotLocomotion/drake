from __future__ import absolute_import, division, print_function

import unittest

from pydrake.all import *


class TestAll(unittest.TestCase):
    def test_symbols(self):
        # Subset of symbols.
        expected_symbols = (
            # autodiffutils
            "AutoDiffXd",
            # common
            "AddResourceSearchPath",
            # parsers
            "PackageMap",
            # rbtree
            "RigidBodyTree",
            # symbolic
            "Variable",
            "Expression",
            # multibody
            "RigidBodyPlant",
            # solvers
            "MathematicalProgram",
            "LinearConstraint",
            # systems
            "BasicVector",
            "LeafSystem",
            "Simulator",
        )
        # Ensure each symbol is exposed as globals from the above import
        # statement.
        for expected_symbol in expected_symbols:
            self.assertTrue(expected_symbol in globals())

    def test_usage_all(self):
        # N.B. Synchronize this with `doc/python_bindings.rst`.
        tree = RigidBodyTree(
            FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"))
        simulator = Simulator(RigidBodyPlant(tree))

    def test_usage_no_all(self):

        def isolated(self):
            # Ensure we have no globals leaking in.
            self.assertEquals(len(globals()), 0)

            # N.B. Synchronize this with `doc/python_bindings.rst`.
            from pydrake.common import FindResourceOrThrow
            from pydrake.multibody.rigid_body_plant import RigidBodyPlant
            from pydrake.rbtree import RigidBodyTree
            from pydrake.systems.analysis import Simulator

            tree = RigidBodyTree(
                FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"))
            simulator = Simulator(RigidBodyPlant(tree))

            return simulator

        # Evaluate without access to current globals.
        # TODO(eric.cousineau): Use `mock.patch.dict` when available.
        # Unfortunately, `isolated.__globals__` references current globals, so
        # we must restore them. `eval(expr, globals)` does not override the
        # scoped globals of the function (as it should be).
        # `isolated.__globals__` is a read-only property as well.
        old_globals = dict(isolated.__globals__)
        isolated.__globals__.clear()
        simulator = isolated(self)
        isolated.__globals__.update(old_globals)
        self.assertTrue(isinstance(simulator, Simulator))


if __name__ == '__main__':
    unittest.main()

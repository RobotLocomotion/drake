# -*- coding: utf-8 -*-

"""
@file
Captures limitations for the present state of the Python bindings for the
lifetime of objects, eventually lock down capabilities as they are introduced.
"""


import unittest
import numpy as np

from pydrake.systems.analysis import (
    Simulator,
    )
from pydrake.systems.framework import (
    DiagramBuilder,
    )
from pydrake.systems.primitives import (
    Adder,
    )
from pydrake.systems.test.test_util import (
    DeleteListenerSystem,
    DeleteListenerVector,
    )


class Info:
    # Tracks if an instance has been deleted.
    def __init__(self):
        self.deleted = False

    def record_deletion(self):
        assert not self.deleted
        self.deleted = True


class TestLifetime(unittest.TestCase):
    def test_basic(self):
        info = Info()
        system = DeleteListenerSystem(info.record_deletion)
        self.assertFalse(info.deleted)
        del system
        self.assertTrue(info.deleted)

    def test_ownership_diagram(self):
        info = Info()
        system = DeleteListenerSystem(info.record_deletion)
        builder = DiagramBuilder()
        # `system` is now owned by `builder`.
        builder.AddSystem(system)
        # `system` is now owned by `diagram`.
        diagram = builder.Build()
        # Delete the builder. Should still be alive.
        del builder
        self.assertFalse(info.deleted)
        # Delete the diagram. Should be dead.
        del diagram
        # Using `py::keep_alive`, `system` will keep `builder` alive after
        # `.AddSystem` is called, and `builder` will keep `diagram` alive after
        # `.Build` is called.
        # Transitively, `system` will keep `builder` alive (as its old owner)
        # and `diagram` (as its new owner, which is kept alive by `builder`).
        self.assertFalse(info.deleted)
        self.assertTrue(system is not None)
        del system
        # Upon removing this reference, everything should have been cleared up.
        # However, since we work around #14355 by inducing a keep_alive cycle,
        # it will not be deleted.
        self.assertFalse(info.deleted)

    def test_ownership_multiple_containers(self):
        info = Info()
        system = DeleteListenerSystem(info.record_deletion)
        builder_1 = DiagramBuilder()
        builder_2 = DiagramBuilder()
        builder_1.AddSystem(system)
        # This is tested in our fork of `pybind11`, but echoed here for when
        # we decide to switch to use `shared_ptr`.
        with self.assertRaises(RuntimeError):
            # This should throw an error from `pybind11`, since two containers
            # are trying to own a unique_ptr-held object.
            builder_2.AddSystem(system)

    def test_ownership_simulator(self):
        info = Info()
        system = DeleteListenerSystem(info.record_deletion)
        simulator = Simulator(system)
        self.assertFalse(info.deleted)
        del simulator
        # Simulator does not own the system.
        self.assertFalse(info.deleted)
        self.assertTrue(system is not None)
        # Now ensure that having a system be alive will keep
        # the system alive (using `py::keep_alive`).
        simulator = Simulator(system)
        del system
        self.assertFalse(info.deleted)
        del simulator
        self.assertTrue(info.deleted)

    def test_ownership_vector(self):
        system = Adder(1, 1)
        context = system.CreateDefaultContext()
        info = Info()
        vector = DeleteListenerVector(info.record_deletion)
        system.get_input_port(0).FixValue(context, vector)
        del context
        # Same as above applications, using `py::keep_alive`.
        self.assertFalse(info.deleted)
        self.assertTrue(vector is not None)
        # Ensure that we do not get segfault behavior when accessing / mutating
        # the values.
        self.assertTrue(np.allclose(vector.get_value(), [0.]))
        vector.get_mutable_value()[:] = [10.]
        self.assertTrue(np.allclose(vector.get_value(), [10.]))

# -*- coding: utf-8 -*-

"""
@file
Captures limitations for the present state of the Python bindings for the
lifetime of objects, eventually lock down capabilities as they are introduced.
"""

import gc
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
        # `system` and `builder` are now joined by a ref_cycle. The c++ system
        # is owned by the c++ builder.
        builder.AddSystem(system)
        # `system`, `builder`, and `diagram` are now joined by ref_cycles. The
        # c++ system is owned by the c++ diagram.
        diagram = builder.Build()
        # Delete the builder. The `system` should still be alive.
        del builder
        self.assertFalse(info.deleted)
        # Delete the diagram. All are still reachable via `system`.
        del diagram
        self.assertFalse(info.deleted)
        self.assertTrue(system is not None)
        del system
        # The system-builder-diagram "life raft" is still alive as a
        # garbage-collectible cycle.
        self.assertFalse(info.deleted)
        # Collect the garbage. The system should now be gone.
        gc.collect()
        self.assertTrue(info.deleted)

    def test_ownership_multiple_containers(self):
        info = Info()
        system = DeleteListenerSystem(info.record_deletion)

        # Add the system to a built diagram.
        builder_1 = DiagramBuilder()
        builder_1.AddSystem(system)
        diagram_1 = builder_1.Build()  # noqa: F841 (unused-variable)

        # Add it again to another diagram. We don't care if the Add fails or
        # the Build fails, so long as one of them does.
        builder_2 = DiagramBuilder()
        with self.assertRaisesRegex(Exception, "already.*different Diagram"):
            builder_2.AddSystem(system)
            builder_2.Build()

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
        self.assertTrue(np.allclose(vector.get_value(), [0.0]))
        vector.get_mutable_value()[:] = [10.0]
        self.assertTrue(np.allclose(vector.get_value(), [10.0]))

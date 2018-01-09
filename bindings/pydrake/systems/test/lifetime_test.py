#!/usr/bin/env python
# -*- coding: utf8 -*-

"""
@file
Captures limitations for the present state of the Python bindings for the
lifetime of objects, eventually lock down capabilities as they are introduced.
"""

from __future__ import print_function

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
from pydrake.systems.test.lifetime_test_util import (
    DeleteListenerSystem,
    DeleteListenerVector,
    )


class Info(object):
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
        # WARNING
        self.assertTrue(info.deleted)
        self.assertTrue(system is not None)

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

    def test_ownership_vector(self):
        system = Adder(1, 1)
        context = system.CreateDefaultContext()
        info = Info()
        vector = DeleteListenerVector(info.record_deletion)
        context.FixInputPort(0, vector)
        del context
        # WARNING
        self.assertTrue(info.deleted)
        self.assertTrue(vector is not None)


assert __name__ == '__main__'
unittest.main()

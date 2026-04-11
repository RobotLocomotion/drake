"""Test event callback bindings against leaks and lifetime problems.

This test forms the callables passed to Declare*Event in several ways, since
the resulting callables have different properties as seen from the Python C
interface.
"""

import gc
import unittest
import weakref

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import LeafSystem


def DeclareAllEvents(system, callback_function):
    system.DeclareInitializationPublishEvent(callback_function)
    system.DeclareInitializationDiscreteUpdateEvent(callback_function)
    system.DeclareInitializationUnrestrictedUpdateEvent(callback_function)
    system.DeclarePeriodicPublishEvent(1.0, 0.5, callback_function)
    system.DeclarePeriodicDiscreteUpdateEvent(1.0, 0.5, callback_function)
    system.DeclarePeriodicUnrestrictedUpdateEvent(1.0, 0.5, callback_function)
    system.DeclarePerStepPublishEvent(callback_function)
    system.DeclarePerStepDiscreteUpdateEvent(callback_function)
    system.DeclarePerStepUnrestrictedUpdateEvent(callback_function)
    system.DeclareForcedPublishEvent(callback_function)
    system.DeclareForcedDiscreteUpdateEvent(callback_function)
    system.DeclareForcedUnrestrictedUpdateEvent(callback_function)


class MethodLeafSystem(LeafSystem):
    """System with callbacks implemented via a bound method."""

    def __init__(self):
        super().__init__()
        self._count = 0
        # These declarations are spelled out on purpose. It turns out that
        # evaluating `self._callback` multiple times triggers unusual pybind11
        # memory behavior that needs to be tested. See UniquelyWrapCallback()
        # in framework_py_systems.cc.
        self.DeclareInitializationPublishEvent(self._callback)
        self.DeclareInitializationDiscreteUpdateEvent(self._callback)
        self.DeclareInitializationUnrestrictedUpdateEvent(self._callback)
        self.DeclarePeriodicPublishEvent(1.0, 0.5, self._callback)
        self.DeclarePeriodicDiscreteUpdateEvent(1.0, 0.5, self._callback)
        self.DeclarePeriodicUnrestrictedUpdateEvent(1.0, 0.5, self._callback)
        self.DeclarePerStepPublishEvent(self._callback)
        self.DeclarePerStepDiscreteUpdateEvent(self._callback)
        self.DeclarePerStepUnrestrictedUpdateEvent(self._callback)
        self.DeclareForcedPublishEvent(self._callback)
        self.DeclareForcedDiscreteUpdateEvent(self._callback)
        self.DeclareForcedUnrestrictedUpdateEvent(self._callback)

    def _callback(self, *args, **kwargs):
        self._count += 1

    def count(self):
        return self._count


a_global_count = 0


def a_global_callback(*args, **kwargs):
    global a_global_count
    a_global_count += 1


class FunctionCallbackLeafSystem(LeafSystem):
    """System with callbacks implemented via a global function."""

    def __init__(self):
        super().__init__()
        DeclareAllEvents(self, a_global_callback)

    def count(self):
        return a_global_count


class ClosureCallbackLeafSystem(LeafSystem):
    """System with callbacks implemented via closures."""

    def __init__(self):
        super().__init__()
        self._count = 0

        def handler(*args, **kwargs):
            self._count += 1

        DeclareAllEvents(self, handler)

    def count(self):
        return self._count


class TestPeriodicEventGarbageCollection(unittest.TestCase):
    """Test that event callbacks don't prevent garbage collection."""

    def do_test_is_collectible(self, system_class):
        system = system_class()
        simulator = Simulator(system)
        # Check that callbacks are callable.
        simulator.Initialize()
        # This count represents 3 kinds of initialization events, plus a
        # per-step publish event.
        self.assertEqual(system.count(), 4)
        simulator.AdvanceTo(0.6)
        # This count has represents these additional events: 3 kinds of
        # periodic events, 2 times 3 kinds of per-step events.
        self.assertEqual(system.count(), 13)
        context = simulator.get_context()
        system.ForcedPublish(context)
        # This count represents an added forced-publish event.
        self.assertEqual(system.count(), 14)
        discrete_state = system.AllocateDiscreteVariables()
        system.CalcForcedDiscreteVariableUpdate(context, discrete_state)
        # This count represents an added forced discrete update event.
        self.assertEqual(system.count(), 15)
        state = context.get_mutable_state()
        system.CalcForcedUnrestrictedUpdate(context, state)
        # This count represents an added forced unrestricted update event.
        self.assertEqual(system.count(), 16)

        spy = weakref.finalize(system, lambda: None)

        del simulator
        del system
        gc.collect()

        # Check the system was removed by garbage collection.
        self.assertFalse(spy.alive)

    def test_method_leaf_system_is_collectible(self):
        """Callbacks via methods don't prevent garbage collection."""
        self.do_test_is_collectible(MethodLeafSystem)

    def test_function_callback_leaf_system_is_collectible(self):
        """Callbacks via global functions don't prevent garbage collection."""
        self.do_test_is_collectible(FunctionCallbackLeafSystem)

    def test_closure_callback_leaf_system_is_collectible(self):
        """Callbacks via closures don't prevent garbage collection."""
        self.do_test_is_collectible(ClosureCallbackLeafSystem)

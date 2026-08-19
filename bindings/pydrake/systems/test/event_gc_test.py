"""Test event callback bindings against leaks and lifetime problems.

This test forms the callables passed to Declare*Event in several ways, since
the resulting callables have different properties as seen from the Python C
interface.
"""

import gc
import unittest
import weakref

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import (
    LeafSystem,
    PublishEvent,
    TriggerType,
    UnrestrictedUpdateEvent,
)


def DeclareAllEvents(system, callback):
    system.DeclareInitializationEvent(PublishEvent(callback))
    system.DeclareInitializationEvent(PublishEvent(system_callback=callback))
    system.DeclareInitializationEvent(
        PublishEvent(TriggerType.kUnknown, callback)
    )
    system.DeclareInitializationEvent(
        PublishEvent(TriggerType.kUnknown, system_callback=callback)
    )
    system.DeclareInitializationEvent(UnrestrictedUpdateEvent(callback))
    system.DeclareInitializationEvent(
        UnrestrictedUpdateEvent(system_callback=callback)
    )

    system.DeclareInitializationPublishEvent(callback)
    system.DeclareInitializationDiscreteUpdateEvent(callback)
    system.DeclareInitializationUnrestrictedUpdateEvent(callback)

    system.DeclarePeriodicEvent(1.0, 0.5, PublishEvent(callback))
    system.DeclarePeriodicEvent(
        1.0, 0.5, PublishEvent(system_callback=callback)
    )
    system.DeclarePeriodicEvent(
        1.0, 0.5, PublishEvent(TriggerType.kUnknown, callback)
    )
    system.DeclarePeriodicEvent(
        1.0,
        0.5,
        PublishEvent(TriggerType.kUnknown, system_callback=callback),
    )
    system.DeclarePeriodicEvent(1.0, 0.5, UnrestrictedUpdateEvent(callback))
    system.DeclarePeriodicEvent(
        1.0, 0.5, UnrestrictedUpdateEvent(system_callback=callback)
    )

    system.DeclarePeriodicPublishEvent(1.0, 0.5, callback)
    system.DeclarePeriodicDiscreteUpdateEvent(1.0, 0.5, callback)
    system.DeclarePeriodicUnrestrictedUpdateEvent(1.0, 0.5, callback)

    system.DeclarePerStepEvent(PublishEvent(callback))
    system.DeclarePerStepEvent(PublishEvent(system_callback=callback))
    system.DeclarePerStepEvent(PublishEvent(TriggerType.kUnknown, callback))
    system.DeclarePerStepEvent(
        PublishEvent(TriggerType.kUnknown, system_callback=callback)
    )
    system.DeclarePerStepEvent(UnrestrictedUpdateEvent(callback))
    system.DeclarePerStepEvent(
        UnrestrictedUpdateEvent(system_callback=callback)
    )

    system.DeclarePerStepPublishEvent(callback)
    system.DeclarePerStepDiscreteUpdateEvent(callback)
    system.DeclarePerStepUnrestrictedUpdateEvent(callback)

    system.DeclareForcedPublishEvent(callback)
    system.DeclareForcedDiscreteUpdateEvent(callback)
    system.DeclareForcedUnrestrictedUpdateEvent(callback)


class MethodLeafSystem(LeafSystem):
    """System with callbacks implemented via a bound method."""

    def __init__(self):
        super().__init__()
        self._count = 0
        # These declarations are spelled out on purpose. It turns out that
        # evaluating `self._callback` multiple times triggers unusual pybind11
        # memory behavior that needs to be tested. See UniquelyWrapCallback()
        # in callback_lifetime_pybind.cc.

        self.DeclareInitializationEvent(PublishEvent(self._callback))
        self.DeclareInitializationEvent(
            PublishEvent(system_callback=self._callback)
        )
        self.DeclareInitializationEvent(
            PublishEvent(TriggerType.kUnknown, self._callback)
        )
        self.DeclareInitializationEvent(
            PublishEvent(TriggerType.kUnknown, system_callback=self._callback)
        )
        self.DeclareInitializationEvent(UnrestrictedUpdateEvent(self._callback))
        self.DeclareInitializationEvent(
            UnrestrictedUpdateEvent(system_callback=self._callback)
        )

        self.DeclareInitializationPublishEvent(self._callback)
        self.DeclareInitializationDiscreteUpdateEvent(self._callback)
        self.DeclareInitializationUnrestrictedUpdateEvent(self._callback)

        self.DeclarePeriodicEvent(1.0, 0.5, PublishEvent(self._callback))
        self.DeclarePeriodicEvent(
            1.0, 0.5, PublishEvent(system_callback=self._callback)
        )
        self.DeclarePeriodicEvent(
            1.0, 0.5, PublishEvent(TriggerType.kUnknown, self._callback)
        )
        self.DeclarePeriodicEvent(
            1.0,
            0.5,
            PublishEvent(TriggerType.kUnknown, system_callback=self._callback),
        )
        self.DeclarePeriodicEvent(
            1.0, 0.5, UnrestrictedUpdateEvent(self._callback)
        )
        self.DeclarePeriodicEvent(
            1.0,
            0.5,
            UnrestrictedUpdateEvent(system_callback=self._callback),
        )

        self.DeclarePeriodicPublishEvent(1.0, 0.5, self._callback)
        self.DeclarePeriodicDiscreteUpdateEvent(1.0, 0.5, self._callback)
        self.DeclarePeriodicUnrestrictedUpdateEvent(1.0, 0.5, self._callback)

        self.DeclarePerStepEvent(PublishEvent(self._callback))
        self.DeclarePerStepEvent(PublishEvent(system_callback=self._callback))
        self.DeclarePerStepEvent(
            PublishEvent(TriggerType.kUnknown, self._callback)
        )
        self.DeclarePerStepEvent(
            PublishEvent(TriggerType.kUnknown, system_callback=self._callback)
        )
        self.DeclarePerStepEvent(UnrestrictedUpdateEvent(self._callback))
        self.DeclarePerStepEvent(
            UnrestrictedUpdateEvent(system_callback=self._callback)
        )

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
        self._spies = []

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
        expected_count = 0
        # This count represents 9 variations of initialization events, plus 5
        # per-step publish events.
        expected_count += 14
        self.assertEqual(system.count(), expected_count)
        simulator.AdvanceTo(0.6)
        # This count has represents these additional events: 9 kinds of
        # periodic events, 2 times 9 kinds of per-step events.
        expected_count += 27
        self.assertEqual(system.count(), expected_count)
        context = simulator.get_context()
        system.ForcedPublish(context)
        # This count represents an added forced-publish event.
        expected_count += 1
        self.assertEqual(system.count(), expected_count)
        discrete_state = system.AllocateDiscreteVariables()
        system.CalcForcedDiscreteVariableUpdate(context, discrete_state)
        # This count represents an added forced discrete update event.
        expected_count += 1
        self.assertEqual(system.count(), expected_count)
        state = context.get_mutable_state()
        system.CalcForcedUnrestrictedUpdate(context, state)
        # This count represents an added forced unrestricted update event.
        expected_count += 1
        self.assertEqual(system.count(), expected_count)

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

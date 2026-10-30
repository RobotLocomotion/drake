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
    WitnessFunctionDirection,
)


def DeclareAllEvents(system, get_callback):
    """Declares one of every kind of event on the given system. Calls
    get_callback() anew for every declaration to obtain the callback; see
    MethodLeafSystem for why that matters.
    """

    def make_event_objects():
        # Always returns new objects, so that no event object is ever passed
        # to more than one declaration.
        return [
            PublishEvent(get_callback()),
            PublishEvent(system_callback=get_callback()),
            PublishEvent(TriggerType.kUnknown, get_callback()),
            PublishEvent(TriggerType.kUnknown, system_callback=get_callback()),
            UnrestrictedUpdateEvent(get_callback()),
            UnrestrictedUpdateEvent(system_callback=get_callback()),
        ]

    for event in make_event_objects():
        system.DeclareInitializationEvent(event)
    system.DeclareInitializationPublishEvent(get_callback())
    system.DeclareInitializationDiscreteUpdateEvent(get_callback())
    system.DeclareInitializationUnrestrictedUpdateEvent(get_callback())

    for event in make_event_objects():
        system.DeclarePeriodicEvent(1.0, 0.5, event)
    system.DeclarePeriodicPublishEvent(1.0, 0.5, get_callback())
    system.DeclarePeriodicDiscreteUpdateEvent(1.0, 0.5, get_callback())
    system.DeclarePeriodicUnrestrictedUpdateEvent(1.0, 0.5, get_callback())

    for event in make_event_objects():
        system.DeclarePerStepEvent(event)
    system.DeclarePerStepPublishEvent(get_callback())
    system.DeclarePerStepDiscreteUpdateEvent(get_callback())
    system.DeclarePerStepUnrestrictedUpdateEvent(get_callback())

    system.DeclareForcedPublishEvent(get_callback())
    system.DeclareForcedDiscreteUpdateEvent(get_callback())
    system.DeclareForcedUnrestrictedUpdateEvent(get_callback())

    # Witness functions are never evaluated by these tests (the system does
    # not report them via DoGetWitnessFunctions), but storing them on the
    # system forms the reference cycles we want to check.
    direction = WitnessFunctionDirection.kCrossesZero
    system._witnesses = [
        system.MakeWitnessFunction("calc", direction, get_callback()),
    ] + [
        system.MakeWitnessFunction("event", direction, get_callback(), event)
        for event in make_event_objects()
    ]


class MethodLeafSystem(LeafSystem):
    """System with callbacks implemented via a bound method."""

    def __init__(self):
        super().__init__()
        self._count = 0
        # It turns out that evaluating `self._callback` multiple times (which
        # creates a new bound method object each time) triggers unusual
        # pybind11 memory behavior that needs to be tested. See
        # UniquelyWrapCallback() in ensure_callback_lifetime_pybind.cc.
        DeclareAllEvents(self, lambda: self._callback)

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
        DeclareAllEvents(self, lambda: a_global_callback)

    def count(self):
        return a_global_count


class ClosureCallbackLeafSystem(LeafSystem):
    """System with callbacks implemented via closures."""

    def __init__(self):
        super().__init__()
        self._count = 0

        def handler(*args, **kwargs):
            self._count += 1

        DeclareAllEvents(self, lambda: handler)

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

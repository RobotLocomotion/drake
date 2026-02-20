import gc
import pprint
import sys
import unittest
import weakref

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import LeafSystem


class MinimalLeafSystem(LeafSystem):
    """Minimal system with a periodic callback."""

    def __init__(self):
        super().__init__()

        self.DeclareInitializationPublishEvent(self._callbackstar)
        self.DeclareInitializationDiscreteUpdateEvent(self._callback2)
        self.DeclarePeriodicPublishEvent(1.0, 0.0, self._callback1)

    def _callback1(self, a):
        pass

    def _callbackstar(self, *args, **kwargs):
        pass

    def _callback2(self, a, b):
        pass


def a_global_callback(*args, **kwargs):
    pass


class FunctionCallbackLeafSystem(LeafSystem):
    """Minimal system with a periodic callback."""

    def __init__(self):
        super().__init__()
        self.DeclareInitializationPublishEvent(a_global_callback)
        self.DeclareInitializationDiscreteUpdateEvent(a_global_callback)
        self.DeclarePeriodicPublishEvent(1.0, 0.0, a_global_callback)


class LambdaCallbackLeafSystem(LeafSystem):
    """Minimal system with a periodic callback."""

    def __init__(self):
        super().__init__()
        self.DeclareInitializationPublishEvent(lambda *args, **kwargs: None)
        self.DeclareInitializationDiscreteUpdateEvent(
            lambda *args, **kwargs: None)
        self.DeclarePeriodicPublishEvent(1.0, 0.0, lambda *args, **kwargs: None)


class TestPeriodicEventGarbageCollection(unittest.TestCase):
    """Test that periodic event callbacks don't prevent garbage collection."""

    def do_test_is_collectible(self, system_class):
        system = system_class()
        simulator = Simulator(system)
        print(f"system referrers"
              f" {pprint.pformat(gc.get_referrers(system))}")
        simulator.AdvanceTo(1.0)
        print(f"system referrers"
              f" {pprint.pformat(gc.get_referrers(system))}")

        spy = weakref.finalize(system, lambda: None)

        del simulator
        del system
        gc.collect()

        self.assertFalse(spy.alive)


    def test_minimal_leaf_system_is_collectible(self):
        """A LeafSystem with a periodic callback must be garbage collectible."""
        self.do_test_is_collectible(MinimalLeafSystem)

    def test_function_callback_leaf_system_is_collectible(self):
        """A LeafSystem with a periodic callback must be garbage collectible."""
        self.do_test_is_collectible(FunctionCallbackLeafSystem)

    def test_lambda_callback_leaf_system_is_collectible(self):
        """A LeafSystem with a periodic callback must be garbage collectible."""
        self.do_test_is_collectible(LambdaCallbackLeafSystem)


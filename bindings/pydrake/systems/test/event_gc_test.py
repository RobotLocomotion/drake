import gc
import pprint
import sys
import unittest
import weakref

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import LeafSystem


class MethodLeafSystem(LeafSystem):
    """System with callbacks implemented via a bound method."""

    def __init__(self):
        super().__init__()
        self.DeclareInitializationPublishEvent(self._callback)
        self.DeclareInitializationDiscreteUpdateEvent(self._callback)
        self.DeclareInitializationUnrestrictedUpdateEvent(self._callback)
        self.DeclarePeriodicPublishEvent(1.0, 0.0, self._callback)
        self.DeclarePeriodicDiscreteUpdateEvent(1.0, 0.0, self._callback)
        self.DeclarePeriodicUnrestrictedUpdateEvent(
            1.0, 0.0, self._callback
        )
        self.DeclarePerStepPublishEvent(self._callback)
        self.DeclarePerStepDiscreteUpdateEvent(self._callback)
        self.DeclarePerStepUnrestrictedUpdateEvent(self._callback)
        self.DeclareForcedPublishEvent(self._callback)
        self.DeclareForcedDiscreteUpdateEvent(self._callback)
        self.DeclareForcedUnrestrictedUpdateEvent(self._callback)

    def _callback(self, *args, **kwargs):
        pass


def a_global_callback(*args, **kwargs):
    pass


class FunctionCallbackLeafSystem(LeafSystem):
    """System with callbacks implemented via a global function."""

    def __init__(self):
        super().__init__()
        self.DeclareInitializationPublishEvent(a_global_callback)
        self.DeclareInitializationDiscreteUpdateEvent(a_global_callback)
        self.DeclareInitializationUnrestrictedUpdateEvent(a_global_callback)
        self.DeclarePeriodicPublishEvent(1.0, 0.0, a_global_callback)
        self.DeclarePeriodicDiscreteUpdateEvent(1.0, 0.0, a_global_callback)
        self.DeclarePeriodicUnrestrictedUpdateEvent(1.0, 0.0, a_global_callback)
        self.DeclarePerStepPublishEvent(a_global_callback)
        self.DeclarePerStepDiscreteUpdateEvent(a_global_callback)
        self.DeclarePerStepUnrestrictedUpdateEvent(a_global_callback)
        self.DeclareForcedPublishEvent(a_global_callback)
        self.DeclareForcedDiscreteUpdateEvent(a_global_callback)
        self.DeclareForcedUnrestrictedUpdateEvent(a_global_callback)


class LambdaCallbackLeafSystem(LeafSystem):
    """System with callbacks implemented via lambdas."""

    def __init__(self):
        super().__init__()
        self.DeclareInitializationPublishEvent(lambda *args, **kwargs: None)
        self.DeclareInitializationDiscreteUpdateEvent(
            lambda *args, **kwargs: None
        )
        self.DeclareInitializationUnrestrictedUpdateEvent(
            lambda *args, **kwargs: None
        )
        self.DeclarePeriodicPublishEvent(1.0, 0.0, lambda *args, **kwargs: None)
        self.DeclarePeriodicDiscreteUpdateEvent(
            1.0, 0.0, lambda *args, **kwargs: None
        )
        self.DeclarePeriodicUnrestrictedUpdateEvent(
            1.0, 0.0, lambda *args, **kwargs: None
        )
        self.DeclarePerStepPublishEvent(lambda *args, **kwargs: None)
        self.DeclarePerStepDiscreteUpdateEvent(lambda *args, **kwargs: None)
        self.DeclarePerStepUnrestrictedUpdateEvent(lambda *args, **kwargs: None)
        self.DeclareForcedPublishEvent(lambda *args, **kwargs: None)
        self.DeclareForcedDiscreteUpdateEvent(lambda *args, **kwargs: None)
        self.DeclareForcedUnrestrictedUpdateEvent(lambda *args, **kwargs: None)


class TestPeriodicEventGarbageCollection(unittest.TestCase):
    """Test that event callbacks don't prevent garbage collection."""

    def do_test_is_collectible(self, system_class):
        system = system_class()
        simulator = Simulator(system)
        # Check that callbacks are callable.
        simulator.AdvanceTo(2.0)

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

    def test_lambda_callback_leaf_system_is_collectible(self):
        """Callbacks via lambdas don't prevent garbage collection."""
        self.do_test_is_collectible(LambdaCallbackLeafSystem)

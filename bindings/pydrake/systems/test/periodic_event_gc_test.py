import gc
import unittest
import weakref

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import LeafSystem


class MinimalLeafSystem(LeafSystem):
    """Minimal system with a periodic callback."""

    def __init__(self):
        super().__init__()
        print("id of callback", hex(id(self._callback)))
        x = self._callback
        print("id of x", hex(id(x)))
        self.DeclarePeriodicPublishEvent(1.0, 0.0, x)

    def _callback(self, context):
        pass


def a_global_callback(context):
    pass

class FunctionCallbackLeafSystem(LeafSystem):
    """Minimal system with a periodic callback."""

    def __init__(self):
        super().__init__()
        print("id of callback", hex(id(a_global_callback)))
        x = a_global_callback
        print("id of x", hex(id(x)))
        self.DeclarePeriodicPublishEvent(1.0, 0.0, x)


class TestPeriodicEventGarbageCollection(unittest.TestCase):
    """Test that periodic event callbacks don't prevent garbage collection."""

    def do_test_is_collectible(self, system_class):
        system = system_class()
        simulator = Simulator(system)
        simulator.AdvanceTo(1.0)

        weak_ref = weakref.ref(system)

        del simulator
        del system
        gc.collect()

        self.assertIsNone(weak_ref())

    @unittest.skip("demand")
    def test_minimal_leaf_system_is_collectible(self):
        """A LeafSystem with a periodic callback must be garbage collectible."""
        self.do_test_is_collectible(MinimalLeafSystem)

    def test_function_callback_leaf_system_is_collectible(self):
        """A LeafSystem with a periodic callback must be garbage collectible."""
        self.do_test_is_collectible(FunctionCallbackLeafSystem)

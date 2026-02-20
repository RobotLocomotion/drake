import gc
import unittest
import weakref

from pydrake.systems.framework import LeafSystem


class MinimalLeafSystem(LeafSystem):
    """Minimal system with a periodic callback."""

    def __init__(self):
        super().__init__()
        self.DeclarePeriodicPublishEvent(1.0, 0.0, self._callback)

    def _callback(self, context):
        pass


class TestPeriodicEventGarbageCollection(unittest.TestCase):
    """Test that periodic event callbacks don't prevent garbage collection."""

    def test_leaf_system_with_periodic_callback_is_collectible(self):
        """A LeafSystem with a periodic callback must be garbage collectible."""
        system = MinimalLeafSystem()
        weak_ref = weakref.ref(system)

        del system
        gc.collect()

        self.assertIsNone(weak_ref())

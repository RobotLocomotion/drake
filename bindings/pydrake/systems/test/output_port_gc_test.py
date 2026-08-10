"""Tests output-port callbacks against leaks and lifetime problems."""

import gc
import unittest
import weakref

from pydrake.systems.framework import BasicVector, LeafSystem


class MethodCallbackLeafSystem(LeafSystem):
    """System with a vector output callback implemented as a bound method."""

    def __init__(self, *, use_model_value):
        super().__init__()
        self._count = 0
        if use_model_value:
            self.DeclareVectorOutputPort(
                "output", BasicVector(1), self._calc_output
            )
        else:
            self.DeclareVectorOutputPort("output", 1, self._calc_output)

    def _calc_output(self, context, output):
        self._count += 1
        output.SetAtIndex(0, self._count)

    def count(self):
        return self._count


class TestOutputPortGarbageCollection(unittest.TestCase):
    def test_method_callback_is_callable_and_collectible(self):
        for use_model_value in (False, True):
            with self.subTest(use_model_value=use_model_value):
                system = MethodCallbackLeafSystem(
                    use_model_value=use_model_value
                )
                context = system.CreateDefaultContext()

                value = system.get_output_port().Eval(context)
                self.assertEqual(value[0], 1)
                self.assertEqual(system.count(), 1)

                spy = weakref.finalize(system, lambda: None)
                del value
                del context
                del system
                gc.collect()

                self.assertFalse(spy.alive)

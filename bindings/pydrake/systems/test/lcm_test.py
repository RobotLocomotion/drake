"""
Test bindings of LCM integration with the Systems framework.
"""
import pydrake.systems.lcm as mut

import collections
from multiprocessing import Process
import time
import unittest

import numpy as np

from robotlocomotion import header_t, quaternion_t

from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.common.value import AbstractValue
from pydrake.lcm import DrakeLcm, Subscriber
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector, DiagramBuilder, LeafSystem
from pydrake.systems.primitives import ConstantVectorSource, LogOutput


# TODO(eric.cousieau): Move this to more generic code when another piece of
# code uses it.
def lcm_to_json(message):
    def helper(thing):
        if type(thing) in (int, float, np.float64, str):
            return thing
        if type(thing) in (list,):
            return list(map(helper, [x for x in thing]))
        result = collections.OrderedDict()
        for field in thing.__slots__:
            value = getattr(thing, field)
            result[field] = helper(value)
        return result
    return helper(message)


class TestSystemsLcm(unittest.TestCase):
    def _model_message(self):
        message = quaternion_t()
        message.w, message.x, message.y, message.z = (1, 2, 3, 4)
        return message

    def _model_value_cpp(self):
        serializer = mut._Serializer_[quaternion_t]()
        model_message = self._model_message()
        model_value = serializer.CreateDefaultValue()
        serializer.Deserialize(model_message.encode(), model_value)
        return model_value

    def _cpp_value_to_py_message(self, value):
        serializer = mut._Serializer_[quaternion_t]()
        raw = serializer.Serialize(value)
        return quaternion_t.decode(raw)

    def assert_lcm_equal(self, actual, expected):
        self.assertIsInstance(actual, type(expected))
        self.assertDictEqual(lcm_to_json(actual), lcm_to_json(expected))

    def assert_lcm_not_equal(self, actual, expected):
        self.assertIsInstance(actual, type(expected))
        self.assertNotEqual(lcm_to_json(actual), lcm_to_json(expected))

    def test_serializer(self):
        dut = mut.PySerializer(quaternion_t)
        model_message = self._model_message()
        value = dut.CreateDefaultValue()
        self.assert_lcm_not_equal(value.get_value(), model_message)
        # Check deserialization.
        dut.Deserialize(model_message.encode(), value)
        self.assert_lcm_equal(value.get_value(), model_message)
        # Check serialization.
        raw = dut.Serialize(value)
        reconstruct = quaternion_t.decode(raw)
        self.assert_lcm_equal(reconstruct, model_message)

    def test_serializer_cpp(self):
        # Tests relevant portions of API.
        model_message = self._model_message()
        model_value = self._model_value_cpp()
        self.assert_lcm_equal(
            self._cpp_value_to_py_message(model_value), model_message)

    def _process_event(self, dut):
        # Use a Simulator to invoke the update event on `dut`.  (Wouldn't it be
        # nice if the Systems API was simple enough that we could apply events
        # without calling a Simulator!)
        simulator = Simulator(dut)
        simulator.AdvanceTo(0.00025)  # Arbitrary positive value.
        return simulator.get_context().Clone()

    def test_subscriber(self):
        lcm = DrakeLcm()
        dut = mut.LcmSubscriberSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm)
        model_message = self._model_message()
        lcm.Publish(channel="TEST_CHANNEL", buffer=model_message.encode())
        lcm.HandleSubscriptions(0)
        context = self._process_event(dut)
        actual_message = dut.get_output_port(0).Eval(context)
        self.assert_lcm_equal(actual_message, model_message)

    def test_subscriber_cpp(self):
        lcm = DrakeLcm()
        dut = mut.LcmSubscriberSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm,
            use_cpp_serializer=True)
        model_message = self._model_message()
        lcm.Publish(channel="TEST_CHANNEL", buffer=model_message.encode())
        lcm.HandleSubscriptions(0)
        context = self._process_event(dut)
        abstract = dut.get_output_port(0).EvalAbstract(context)
        actual_message = self._cpp_value_to_py_message(abstract)
        self.assert_lcm_equal(actual_message, model_message)

    def test_subscriber_wait_for_message(self):
        """Checks how `WaitForMessage` works without Python threads."""
        lcm = DrakeLcm()
        sub = mut.LcmSubscriberSystem.Make("TEST_LOOP", header_t, lcm)
        value = AbstractValue.Make(header_t())
        for old_message_count in range(3):
            message = header_t()
            message.utime = old_message_count + 1
            lcm.Publish("TEST_LOOP", message.encode())
            for attempt in range(10):
                new_count = sub.WaitForMessage(
                    old_message_count, value, timeout=0.02)
                if new_count > old_message_count:
                    break
                lcm.HandleSubscriptions(0)
            self.assertEqual(value.get_value().utime, old_message_count + 1)

    def _fix_and_publish(self, dut, value):
        context = dut.CreateDefaultContext()
        dut.get_input_port(0).FixValue(context, value)
        dut.Publish(context)

    def test_publisher(self):
        lcm = DrakeLcm()
        dut = mut.LcmPublisherSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm,
            publish_period=0.1)
        subscriber = Subscriber(lcm, "TEST_CHANNEL", quaternion_t)
        model_message = self._model_message()
        self._fix_and_publish(dut, AbstractValue.Make(model_message))
        lcm.HandleSubscriptions(0)
        self.assert_lcm_equal(subscriber.message, model_message)

    def test_publisher_cpp(self):
        lcm = DrakeLcm()
        dut = mut.LcmPublisherSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm,
            use_cpp_serializer=True)
        subscriber = Subscriber(lcm, "TEST_CHANNEL", quaternion_t)
        model_message = self._model_message()
        model_value = self._model_value_cpp()
        self._fix_and_publish(dut, model_value)
        lcm.HandleSubscriptions(0)
        self.assert_lcm_equal(subscriber.message, model_message)

    def test_connect_lcm_scope(self):
        builder = DiagramBuilder()
        source = builder.AddSystem(ConstantVectorSource(np.zeros(4)))
        mut.ConnectLcmScope(src=source.get_output_port(0),
                            channel="TEST_CHANNEL",
                            builder=builder,
                            lcm=DrakeLcm())

    def test_lcm_interface_system_diagram(self):
        # First, check the class doc.
        self.assertIn(
            "only inherits from LeafSystem", mut.LcmInterfaceSystem.__doc__)
        # Next, construct a diagram and add both the interface system and
        # a subscriber.
        builder = DiagramBuilder()
        lcm = DrakeLcm()
        lcm_system = builder.AddSystem(mut.LcmInterfaceSystem(lcm=lcm))
        # Create subscriber in the diagram.
        subscriber = builder.AddSystem(mut.LcmSubscriberSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm))
        diagram = builder.Build()
        simulator = Simulator(diagram)
        simulator.Initialize()
        # Publish test message.
        model_message = self._model_message()
        lcm.Publish("TEST_CHANNEL", model_message.encode())
        # Simulate to a non-zero time to ensure the subscriber picks up the
        # message.
        eps = np.finfo(float).eps
        simulator.AdvanceTo(eps)
        # Ensure that we have what we want.
        context = subscriber.GetMyContextFromRoot(
            simulator.get_mutable_context())
        actual_message = subscriber.get_output_port(0).Eval(context)
        self.assert_lcm_equal(actual_message, model_message)

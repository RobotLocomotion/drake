"""
Test bindings of LCM integration with the Systems framework.
"""
import pydrake.systems.lcm as mut

import collections
from multiprocessing import Process
import time
import unittest

import numpy as np

from drake import lcmt_header, lcmt_quaternion
import drake as drake_lcmtypes

from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.common.value import AbstractValue
from pydrake.lcm import DrakeLcm, Subscriber
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector, DiagramBuilder, LeafSystem
from pydrake.systems.primitives import ConstantVectorSource


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
        message = lcmt_quaternion()
        message.w, message.x, message.y, message.z = (1, 2, 3, 4)
        return message

    def _model_value_cpp(self):
        serializer = mut._Serializer_[lcmt_quaternion]()
        model_message = self._model_message()
        model_value = serializer.CreateDefaultValue()
        serializer.Deserialize(model_message.encode(), model_value)
        return model_value

    def _cpp_value_to_py_message(self, value):
        serializer = mut._Serializer_[lcmt_quaternion]()
        raw = serializer.Serialize(value)
        return lcmt_quaternion.decode(raw)

    def assert_lcm_equal(self, actual, expected):
        self.assertIsInstance(actual, type(expected))
        self.assertDictEqual(lcm_to_json(actual), lcm_to_json(expected))

    def assert_lcm_not_equal(self, actual, expected):
        self.assertIsInstance(actual, type(expected))
        self.assertNotEqual(lcm_to_json(actual), lcm_to_json(expected))

    def test_serializer(self):
        dut = mut.PySerializer(lcmt_quaternion)
        model_message = self._model_message()
        value = dut.CreateDefaultValue()
        self.assert_lcm_not_equal(value.get_value(), model_message)
        # Check deserialization.
        dut.Deserialize(model_message.encode(), value)
        self.assert_lcm_equal(value.get_value(), model_message)
        # Check serialization.
        raw = dut.Serialize(value)
        reconstruct = lcmt_quaternion.decode(raw)
        self.assert_lcm_equal(reconstruct, model_message)
        # Check cloning.
        cloned_dut = dut.Clone()
        fresh_value = dut.CreateDefaultValue().get_value()
        self.assertIsInstance(fresh_value, lcmt_quaternion)

    def test_serializer_cpp(self):
        # Tests relevant portions of API.
        model_message = self._model_message()
        model_value = self._model_value_cpp()
        self.assert_lcm_equal(
            self._cpp_value_to_py_message(model_value), model_message)

    def test_serializer_cpp_clone(self):
        serializer = mut._Serializer_[lcmt_quaternion]()
        serializer.Clone().CreateDefaultValue()

    def test_all_serializers_exist(self):
        """Checks that all of Drake's Python LCM messages have a matching C++
        serializer bound for use by LcmPublisherSystem.
        """
        # The drake_lcm_py_library() in drake/lcmtypes/BUILD.bazel generates
        # a module __init__.py that enumerates all Drake Python LCM messages.
        # Fetch that module's list of message classes.
        all_message_classes = [
            getattr(drake_lcmtypes, name) for name in dir(drake_lcmtypes)
            if any([name.startswith("lcmt_"),
                    name.startswith("experimental_lcmt_")])
        ]
        self.assertGreater(len(all_message_classes), 1)
        # Confirm that each message class is partnered with the definition of a
        # C++ serializer in lcm_py_bind_cpp_serializers.cc.
        lcm = DrakeLcm()
        for message_class in all_message_classes:
            # Check that the Python message class is a valid template value.
            serializer = mut._Serializer_[message_class]
            self.assertIsNotNone(serializer)
            # Confirm that we can actually instantiate a publisher that takes
            # the matching C++ message on its input port.
            mut.LcmPublisherSystem.Make(
                channel="TEST_CHANNEL", lcm_type=message_class, lcm=lcm,
                use_cpp_serializer=True)

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
            channel="TEST_CHANNEL", lcm_type=lcmt_quaternion, lcm=lcm)
        model_message = self._model_message()
        lcm.Publish(channel="TEST_CHANNEL", buffer=model_message.encode())
        lcm.HandleSubscriptions(0)
        context = self._process_event(dut)
        actual_message = dut.get_output_port(0).Eval(context)
        self.assert_lcm_equal(actual_message, model_message)

    def test_subscriber_cpp(self):
        lcm = DrakeLcm()
        dut = mut.LcmSubscriberSystem.Make(
            channel="TEST_CHANNEL", lcm_type=lcmt_quaternion, lcm=lcm,
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
        sub = mut.LcmSubscriberSystem.Make("TEST_LOOP", lcmt_header, lcm)
        value = AbstractValue.Make(lcmt_header())
        for old_message_count in range(3):
            message = lcmt_header()
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
            channel="TEST_CHANNEL", lcm_type=lcmt_quaternion, lcm=lcm,
            publish_period=0.1)
        subscriber = Subscriber(lcm, "TEST_CHANNEL", lcmt_quaternion)
        model_message = self._model_message()
        self._fix_and_publish(dut, AbstractValue.Make(model_message))
        lcm.HandleSubscriptions(0)
        self.assert_lcm_equal(subscriber.message, model_message)

    def test_publisher_cpp(self):
        lcm = DrakeLcm()
        dut = mut.LcmPublisherSystem.Make(
            channel="TEST_CHANNEL", lcm_type=lcmt_quaternion, lcm=lcm,
            use_cpp_serializer=True)
        subscriber = Subscriber(lcm, "TEST_CHANNEL", lcmt_quaternion)
        model_message = self._model_message()
        model_value = self._model_value_cpp()
        self._fix_and_publish(dut, model_value)
        lcm.HandleSubscriptions(0)
        self.assert_lcm_equal(subscriber.message, model_message)

    class PythonMessageSource(LeafSystem):
        """A source system whose output port contains a Python lcmt_header."""

        def __init__(self):
            LeafSystem.__init__(self)
            self.DeclareAbstractOutputPort(
                "output", self.AllocateOutput, self.CalcOutput)

        def AllocateOutput(self):
            return AbstractValue.Make(lcmt_header())

        def CalcOutput(self, context, output):
            message = output.get_mutable_value()
            message.utime = int(context.get_time() * 1e6)
            message.frame_name = "frame_name"

    def test_diagram_publisher(self):
        """Acceptance tests that a Python LeafSystem is able to output LCM
        messages for LcmPublisherSystem to transmit.
        """
        lcm = DrakeLcm()
        builder = DiagramBuilder()
        source = builder.AddSystem(TestSystemsLcm.PythonMessageSource())
        publisher = builder.AddSystem(
            mut.LcmPublisherSystem.Make(
                channel="LCMT_HEADER",
                lcm_type=lcmt_header,
                lcm=lcm,
                publish_period=0.05))
        builder.Connect(source.get_output_port(), publisher.get_input_port())
        diagram = builder.Build()
        diagram.Publish(diagram.CreateDefaultContext())

    def test_lcm_scope(self):
        builder = DiagramBuilder()
        source = builder.AddSystem(ConstantVectorSource(np.zeros(4)))
        scope, publisher = mut.LcmScopeSystem.AddToBuilder(
            builder=builder,
            lcm=DrakeLcm(),
            signal=source.get_output_port(0),
            channel="TEST_CHANNEL",
            publish_period=0.001)
        self.assertIsInstance(scope, mut.LcmScopeSystem)
        self.assertIsInstance(publisher, mut.LcmPublisherSystem)

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
            channel="TEST_CHANNEL", lcm_type=lcmt_quaternion, lcm=lcm))
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

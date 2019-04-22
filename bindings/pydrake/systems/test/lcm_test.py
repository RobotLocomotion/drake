import pydrake.systems.lcm as mut

import collections
from multiprocessing import Process
import time
import unittest

import numpy as np
from six import text_type as unicode

from robotlocomotion import header_t, quaternion_t

from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.lcm import DrakeLcm, DrakeMockLcm, Subscriber
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import (
    AbstractValue, BasicVector, DiagramBuilder, LeafSystem)
from pydrake.systems.primitives import ConstantVectorSource, LogOutput


# TODO(eric.cousieau): Move this to more generic code when another piece of
# code uses it.
def lcm_to_json(message):
    def helper(thing):
        if type(thing) in (int, float, np.float64, str, unicode):
            return thing
        if type(thing) in (list,):
            return map(helper, [x for x in thing])
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
        lcm = DrakeMockLcm()
        dut = mut.LcmSubscriberSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm)
        model_message = self._model_message()
        lcm.Publish(channel="TEST_CHANNEL", buffer=model_message.encode())
        lcm.HandleSubscriptions(0)
        context = self._process_event(dut)
        actual_message = dut.get_output_port(0).Eval(context)
        self.assert_lcm_equal(actual_message, model_message)

    def test_subscriber_cpp(self):
        lcm = DrakeMockLcm()
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
        """Ensures that `WaitForMessage` threading works in a Python workflow
        that is not threaded."""
        # N.B. This will fail with `threading`. See below for using
        # `multithreading`.
        lcm = DrakeLcm("memq://")
        with catch_drake_warnings(expected_count=1):
            lcm.StartReceiveThread()
        sub = mut.LcmSubscriberSystem.Make("TEST_LOOP", header_t, lcm)
        value = AbstractValue.Make(header_t())
        for i in range(3):
            message = header_t()
            message.utime = i
            lcm.Publish("TEST_LOOP", message.encode())
            sub.WaitForMessage(i, value)
            self.assertEqual(value.get_value().utime, i)

    def test_subscriber_wait_for_message_with_timeout(self):
        """Confirms that the subscriber times out."""
        lcm = DrakeLcm("memq://")
        with catch_drake_warnings(expected_count=1):
            lcm.StartReceiveThread()
        sub = mut.LcmSubscriberSystem.Make("TEST_LOOP", header_t, lcm)
        sub.WaitForMessage(0, timeout=0.02)
        # This test fails if the test hangs.

    def _fix_and_publish(self, dut, value):
        context = dut.CreateDefaultContext()
        context.FixInputPort(0, value)
        dut.Publish(context)

    def test_publisher(self):
        lcm = DrakeMockLcm()
        dut = mut.LcmPublisherSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm,
            publish_period=0.1)
        subscriber = Subscriber(lcm, "TEST_CHANNEL", quaternion_t)
        model_message = self._model_message()
        self._fix_and_publish(dut, AbstractValue.Make(model_message))
        lcm.HandleSubscriptions(0)
        self.assert_lcm_equal(subscriber.message, model_message)

    def test_publisher_cpp(self):
        lcm = DrakeMockLcm()
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
                            lcm=DrakeMockLcm())

    def test_utime_to_seconds(self):
        msg = header_t()
        msg.utime = int(1e6)
        with catch_drake_warnings(expected_count=1):
            dut = mut.PyUtimeMessageToSeconds(header_t)
        t_sec = dut.GetTimeInSeconds(AbstractValue.Make(msg))
        self.assertEqual(t_sec, 1)

    def test_lcm_driven_loop(self):
        """Duplicates the test logic in `lcm_driven_loop_test.cc`."""
        lcm_url = "udpm://239.255.76.67:7669"
        t_start = 3.
        t_end = 10.

        def publish_loop():
            # Publishes a set of messages for the driven loop. This should be
            # run from a separate process.
            # N.B. Because of this, care should be taken not to share C++
            # objects between process boundaries.
            t = t_start
            while t <= t_end:
                message = header_t()
                message.utime = int(1e6 * t)
                lcm.Publish("TEST_LOOP", message.encode())
                time.sleep(0.1)
                t += 1

        class DummySys(LeafSystem):
            # Converts message to time in seconds.
            def __init__(self):
                LeafSystem.__init__(self)
                self.DeclareAbstractInputPort(
                    "header_t", AbstractValue.Make(header_t))
                self.DeclareVectorOutputPort(
                    BasicVector(1), self._calc_output)

            def _calc_output(self, context, output):
                message = self.EvalAbstractInput(context, 0).get_value()
                y = output.get_mutable_value()
                y[:] = message.utime / 1e6

        # Construct diagram for LcmDrivenLoop.
        lcm = DrakeLcm(lcm_url)
        with catch_drake_warnings(expected_count=1):
            utime = mut.PyUtimeMessageToSeconds(header_t)
        sub = mut.LcmSubscriberSystem.Make("TEST_LOOP", header_t, lcm)
        builder = DiagramBuilder()
        builder.AddSystem(sub)
        dummy = builder.AddSystem(DummySys())
        builder.Connect(sub.get_output_port(0), dummy.get_input_port(0))
        logger = LogOutput(dummy.get_output_port(0), builder)
        logger.set_forced_publish_only()
        diagram = builder.Build()
        with catch_drake_warnings(expected_count=1):
            dut = mut.LcmDrivenLoop(diagram, sub, None, lcm, utime)
        dut.set_publish_on_every_received_message(True)

        # N.B. Use `multiprocessing` instead of `threading` so that we may
        # avoid issues with GIL deadlocks.
        publish_proc = Process(target=publish_loop)
        publish_proc.start()
        # Initialize to first message.
        first_msg = dut.WaitForMessage()
        dut.get_mutable_context().SetTime(utime.GetTimeInSeconds(first_msg))
        # Run to desired amount. (Anything more will cause interpreter to
        # "freeze".)
        dut.RunToSecondsAssumingInitialized(t_end)
        publish_proc.join()

        # Check expected values.
        log_t_expected = np.array([4, 5, 6, 7, 8, 9])
        log_t = logger.sample_times()
        self.assertTrue(np.allclose(log_t_expected, log_t))
        log_y = logger.data()
        self.assertTrue(np.allclose(log_t_expected, log_y))

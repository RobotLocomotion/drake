import pydrake.systems.lcm as mut

import collections
import unittest

import numpy as np
from six import text_type as unicode

from robotlocomotion import quaternion_t

from pydrake.lcm import DrakeMockLcm
from pydrake.systems.framework import AbstractValue, DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource


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

    def assert_lcm_equal(self, actual, expected):
        self.assertIsInstance(actual, type(expected))
        self.assertDictEqual(lcm_to_json(actual), lcm_to_json(expected))

    def assert_lcm_not_equal(self, actual, expected):
        self.assertIsInstance(actual, type(expected))
        self.assertNotEqual(lcm_to_json(actual), lcm_to_json(expected))

    def test_serializer(self):
        dut = mut.PySerializer(quaternion_t)
        model = self._model_message()
        value = dut.CreateDefaultValue()
        self.assert_lcm_not_equal(value.get_value(), model)
        # Check deserialization.
        dut.Deserialize(model.encode(), value)
        self.assert_lcm_equal(value.get_value(), model)
        # Check serialization.
        raw = dut.Serialize(value)
        reconstruct = quaternion_t.decode(raw)
        self.assert_lcm_equal(reconstruct, model)

    def test_subscriber(self):
        lcm = DrakeMockLcm()
        dut = mut.LcmSubscriberSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm)
        model = self._model_message()
        lcm.InduceSubscriberCallback(
            channel="TEST_CHANNEL", buffer=model.encode())
        context = dut.CreateDefaultContext()
        output = dut.AllocateOutput()
        dut.CalcOutput(context, output)
        actual = output.get_data(0).get_value()
        self.assert_lcm_equal(actual, model)

    def test_publisher(self):
        lcm = DrakeMockLcm()
        dut = mut.LcmPublisherSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm,
            publish_period=0.1)
        model = self._model_message()
        context = dut.CreateDefaultContext()
        context.FixInputPort(0, AbstractValue.Make(model))
        dut.PublishInputAsLcmMessage(context)
        raw = lcm.get_last_published_message("TEST_CHANNEL")
        value = quaternion_t.decode(raw)
        self.assert_lcm_equal(value, model)

    def test_connect_lcm_scope(self):
        builder = DiagramBuilder()
        source = builder.AddSystem(ConstantVectorSource(np.zeros(4)))
        mut.ConnectLcmScope(src=source.get_output_port(0),
                            channel="TEST_CHANNEL",
                            builder=builder,
                            lcm=DrakeMockLcm())

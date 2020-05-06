# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.
from pydrake.common.value import AbstractValue


class PySerializer(SerializerInterface):
    """Provides a Python implementation of `SerializerInterface` for use
    with `LcmPublisherSystem` and `LcmSubscriberSystem` when the given
    `lcm_type` is a Python object (not a C++ object).
    """
    def __init__(self, lcm_type):
        SerializerInterface.__init__(self)
        self._lcm_type = lcm_type

    def CreateDefaultValue(self):
        return AbstractValue.Make(self._lcm_type())

    def Deserialize(self, buffer, abstract_value):
        message = self._lcm_type.decode(buffer)
        abstract_value.set_value(message)

    def Serialize(self, abstract_value):
        assert isinstance(abstract_value, AbstractValue)
        message = abstract_value.get_value()
        assert isinstance(message, self._lcm_type)
        return message.encode()


@staticmethod
def _make_lcm_subscriber(channel, lcm_type, lcm, use_cpp_serializer=False):
    """Convenience to create an LCM subscriber system with a concrete type.

    Args:
        channel: LCM channel name.
        lcm_type: Python class generated by lcmgen.
        lcm: LCM service instance.
        use_cpp_serializer: Use C++ serializer to interface with LCM converter
            systems that are implemented in C++. LCM types must be registered
            in C++ via `BindCppSerializer`.
    """
    # N.B. This documentation is actually public, as it is assigned to classes
    # below as a static class method.
    if not use_cpp_serializer:
        serializer = PySerializer(lcm_type)
    else:
        serializer = _Serializer_[lcm_type]()
    return LcmSubscriberSystem(channel, serializer, lcm)


@staticmethod
def _make_lcm_publisher(
        channel, lcm_type, lcm, publish_period=0.0, use_cpp_serializer=False):
    """Convenience to create an LCM publisher system with a concrete type.

    Args:
        channel: LCM channel name.
        lcm_type: Python class generated by lcmgen.
        lcm: LCM service instance.
        publish_period: System's publish period (in seconds). Default is 0.
        use_cpp_serializer: Use C++ serializer to interface with LCM converter
            systems that are implemented in C++. LCM types must be registered
            in C++ via `BindCppSerializer`.
    """
    if not use_cpp_serializer:
        serializer = PySerializer(lcm_type)
    else:
        serializer = _Serializer_[lcm_type]()
    return LcmPublisherSystem(channel, serializer, lcm, publish_period)


LcmSubscriberSystem.Make = _make_lcm_subscriber
LcmPublisherSystem.Make = _make_lcm_publisher

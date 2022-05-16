#pragma once

/**
@file
LCM types in C++ and Python are implemented differently, and thus cannot simply
be passed back and forth between the two languages. The simplest method to
communicate the structures is to expose the serialization of C++ `Value<>`s to
Python, so that Python can serialize and deserialize between C++ and Python
types.

For more usages in Python, see the following in `_lcm_extra.py`:
- _make_lcm_subscriber / LcmSubscriberSystem.Make
- _make_lcm_publisher / LcmPublisherSystem.Make
*/

namespace drake {
namespace pydrake {
namespace pysystems {
namespace pylcm {

// TODO(eric.cousineau): Consider providing Starlark to codegen binding code.
void BindCppSerializers();

}  // namespace pylcm
}  // namespace pysystems
}  // namespace pydrake
}  // namespace drake

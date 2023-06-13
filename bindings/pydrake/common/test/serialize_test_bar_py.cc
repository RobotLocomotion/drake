#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/common/test/serialize_test_foo_py.h"
#include "drake/common/name_value.h"

namespace drake {
namespace pydrake {
namespace test {

// A compound serializable struct for unit testing.
// Bar has a member field of type Foo.
struct Bar {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(bar_value));
  }
  Foo bar_value{};
};

PYBIND11_MODULE(serialize_test_bar, m) {
  // N.B. We're _supposed_ to do `py::module::import("foo")` here because we
  // have a serialized member field of type Foo, but instead we've injected a
  // bug into this binding by omitting it. The regression test will confirm
  // that `import bar` raises an exception.

  py::class_<Bar> cls(m, "Bar");
  DefAttributesUsingSerialize(&cls);
}

}  // namespace test
}  // namespace pydrake
}  // namespace drake

#include "drake/bindings/pydrake/common/type_safe_index_pybind.h"

// @file
// Provides bindings for testing the behavior of
// `type_safe_index_pybind.h`. The main test code is in
// `type_safe_index_test.py`.

namespace drake {
namespace pydrake {

PYBIND11_MODULE(type_safe_index_test_util, m) {
  struct Tag {};
  using Index = TypeSafeIndex<Tag>;
  BindTypeSafeIndex<Index>(m, "Index");

  m.def("pass_thru_int", [](int x) { return x; });
  m.def("pass_thru_index", [](Index x) { return x; });

  struct OtherTag {};
  using OtherIndex = TypeSafeIndex<OtherTag>;
  BindTypeSafeIndex<OtherIndex>(m, "OtherIndex");
}

}  // namespace pydrake
}  // namespace drake

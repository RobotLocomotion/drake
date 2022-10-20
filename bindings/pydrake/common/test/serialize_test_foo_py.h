#pragma once

#include "drake/common/name_value.h"

namespace drake {
namespace pydrake {
namespace test {

// A simple serializable struct for unit testing.
struct Foo {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(foo_value));
  }
  int foo_value{};
};

}  // namespace test
}  // namespace pydrake
}  // namespace drake

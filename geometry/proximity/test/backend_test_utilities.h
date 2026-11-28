#pragma once

#include <utility>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/coal_backend.h"
#include "drake/geometry/proximity/collision_backend.h"
#include "drake/geometry/proximity/fcl_backend.h"

namespace drake {
namespace geometry {
namespace internal {

/* Invokes `test` with the backend policy selected for this process. The
 callable must have a templated call operator whose sole template parameter is
 the backend policy. */
template <typename Test>
void RunForConfiguredBackend(Test&& test) {
  switch (GetDefaultCollisionBackend()) {
    case CollisionBackend::kFcl:
      std::forward<Test>(test).template operator()<FclBackend>();
      return;
    case CollisionBackend::kCoal:
      std::forward<Test>(test).template operator()<CoalBackend>();
      return;
  }
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake

/* Defines a normal GoogleTest backed by a function template. The function
 body is parameterized on `Backend`; the GoogleTest body selects and invokes
 one specialization at run time. */
#define DRAKE_BACKEND_GTEST(test_suite, test_name)        \
  template <typename Backend>                             \
  void test_suite##_##test_name();                        \
  GTEST_TEST(test_suite, test_name) {                     \
    ::drake::geometry::internal::RunForConfiguredBackend( \
        []<typename Backend>() {                          \
          test_suite##_##test_name<Backend>();            \
        });                                               \
  }                                                       \
  template <typename Backend>                             \
  void test_suite##_##test_name()

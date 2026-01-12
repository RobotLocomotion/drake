#include <gtest/gtest.h>

#include "drake/geometry/render_gl/internal_loaders.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {
namespace {

// Confirm that the functions exist (and can be linked), but throw.
GTEST_TEST(NoLoadersTest, Exceptions) {
  EXPECT_THROW(GladLoaderLoadEgl(), std::exception);
  EXPECT_THROW(GladLoaderLoadGlx(), std::exception);
}

}  // namespace
}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake

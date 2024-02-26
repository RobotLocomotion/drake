#include "drake/geometry/render_gl/internal_buffer_dim.h"

#include <unordered_set>

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {
namespace {

GTEST_TEST(BufferDimTest, Construction) {
  const int w = 127;
  const int h = 328;
  const BufferDim buffer(w, h);

  EXPECT_EQ(buffer.width(), w);
  EXPECT_EQ(buffer.height(), h);
}

GTEST_TEST(BufferDimTest, Equality) {
  const int w = 127;
  const int h = 328;
  const BufferDim buffer(w, h);
  const BufferDim same(w, h);
  const BufferDim different1(w + 1, h);
  const BufferDim different2(w, h + 1);
  const BufferDim different3(w + 1, h + 1);

  EXPECT_EQ(buffer, same);
  // BufferDim has operator== (implicitly used by EXPECT_EQ) but _not_
  // operator != used by EXPECT_NE.
  EXPECT_FALSE(buffer == different1);
  EXPECT_FALSE(buffer == different2);
  EXPECT_FALSE(buffer == different3);
}

GTEST_TEST(BufferDimTest, HashableKey) {
  const int w = 127;
  const int h = 328;
  const BufferDim buffer(w, h);
  const BufferDim same(w, h);

  std::unordered_set<BufferDim> buffers;

  EXPECT_FALSE(buffers.contains(buffer));
  buffers.insert(buffer);
  EXPECT_TRUE(buffers.contains(buffer));
  EXPECT_TRUE(buffers.contains(same));
  buffers.insert(same);
  EXPECT_TRUE(buffers.contains(buffer));
  EXPECT_TRUE(buffers.contains(same));

  // Show that different buffers aren't mistaken for `buffer`.
  EXPECT_FALSE(buffers.contains(BufferDim(w, h + 1)));
  EXPECT_FALSE(buffers.contains(BufferDim(w + 1, h)));
  EXPECT_FALSE(buffers.contains(BufferDim(w + 1, h + 1)));
}

}  // namespace
}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake

#pragma once

#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

// Type trait for TestTiff image data underlying type, similar to ImageTraits.
template <int bit_depth>
struct TestTiffTraits {
  // Compile error for unspecialized use.
};

template <>
struct TestTiffTraits<8> {
  using ChannelType = uint8_t;
};

template <>
struct TestTiffTraits<16> {
  using ChannelType = uint16_t;
};

template <>
struct TestTiffTraits<32> {
  using ChannelType = float;
  static_assert(sizeof(ChannelType) == 4, "Expected float to be 4 bytes.");
};

/* On construction, create a TIFF with some arbitrary but non-zero values.  This
 class is primarily used for generating fake depth images, so a simple scheme
 of increasing each pixel / channel value by one is used.  For example, with
 an RGB image, the pixels start as:

 - (0, 0): [0, 1, 2]
 - (0, 1): [3, 4, 5]

 For a gray image,

 - (0, 0): 0
 - (0, 1): 1

 For uint8_t and uint16_t, the value inserted will roll back to 0 via a modulus
 with the maximum value for the ChannelType (e.g., % 255 for uint8_t).  For
 float, no bounds checking is enforced as the size of image being generated
 will not overflow pas the range of float unless unreasonably large images
 are requrested to be generated. */
template <int channels, int bit_depth>
class TestTiff {
 public:
  static_assert(channels == 1 || channels == 3,
                "TestTiff <channels> must be 1 (gray) or 3 (RGB).");
  static_assert(bit_depth == 8 || bit_depth == 16 || bit_depth == 32,
                "TestTiff <bit_depth> must be 8, 16, or 32");
  static_assert((channels == 1 && bit_depth == 16) ||      // 16 bit gray
                    (channels == 1 && bit_depth == 32) ||  // 32 bit gray
                    (channels == 3 && bit_depth == 32) ||  // 32 bit RGB
                    (channels == 1 && bit_depth == 8),     // 8 bit gray
                "TestTiff: unsupported <channels, bit_depth> combination.");

  // Data type to allocate for memory, either uint8_t or uint16_t.
  using ChannelType = typename TestTiffTraits<bit_depth>::ChannelType;
  static_assert(
      std::is_same_v<ChannelType, uint8_t> ||
          std::is_same_v<ChannelType, uint16_t> ||
          std::is_same_v<ChannelType, float>,
      "TestTiff: only uint8_t, uint16_t, and 32 bit float supported.");

  /* Creates a TIFF image at the specified path with dimensions width x height,
   generating the pattern described on the class documentation.  User is
   responsible for deleting the file after it is created. */
  TestTiff(const std::string& path, int width, int height);

  // Returns the value for the provided loop index
  static ChannelType TestTiffValue(int idx);

  /* If the image coordinates are wrong for loading, this test will fail with
   actual test information being reported in the failure. */
  template <class DrakeImage>
  static void CornerCheck(const DrakeImage& image) {
    using DestType = typename DrakeImage::T;
    EXPECT_EQ(image.at(0, 0)[0], static_cast<DestType>(0));
  }

  /* Tests all the pixels in the test image, but error reporting is not helpful
   in identifying what was wrong where. */
  template <class DrakeImage>
  static void FullImageCheck(const DrakeImage& image) {
    int n_good{0};
    const int n_expected = image.width() * image.height();
    float idx = 0.0f;
    for (int j = 0; j < image.height(); ++j) {
      for (int i = 0; i < image.width(); ++i) {
        n_good += static_cast<int>(image.at(i, j)[0] == TestTiffValue(idx++));
      }
    }
    EXPECT_EQ(n_good, n_expected);
  }

  const std::string path_;
  const int width_;
  const int height_;
};

// Instantiate them once in the source file.
extern template class TestTiff<1, 8>;   // TestTiffGray8
extern template class TestTiff<1, 16>;  // TestTiffGray16
extern template class TestTiff<1, 32>;  // TestTiffGray32
extern template class TestTiff<3, 32>;  // TestTiffRgb32

// Declare nice type names for test cases to use.
using TestTiffGray8 = TestTiff<1, 8>;
using TestTiffGray16 = TestTiff<1, 16>;
using TestTiffGray32 = TestTiff<1, 32>;
using TestTiffRgb32 = TestTiff<3, 32>;

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake

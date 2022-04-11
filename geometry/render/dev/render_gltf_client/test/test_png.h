#pragma once

#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

// Type trait for TestPng image data underlying type, similar to ImageTraits.
template <int bit_depth>
struct TestPngTraits {
  // Compile error for unspecialized use.
};

template <>
struct TestPngTraits<8> {
  using ChannelType = uint8_t;
};

template <>
struct TestPngTraits<16> {
  using ChannelType = uint16_t;
};

/* On construction, create a PNG with some arbitrary but non-zero values.  Only
 supports creating single channel, three channel (RGB), or 4 channel (RGBA) PNG
 images.  To help validate that the image coordinate system is being set
 correctly (which corner is "lower left") we create an "image" that has
 "increasing values" (modulo the maximum value for ChannelType) by pixel
 channel.  For example, with an RGB image, the pixels start as:

 - (0, 0): [kRedStart, kGreenStart, kBlueStart]
 - (0, 1): [(kRedStart+1) % 255, (kGreenStart+1) % 255, (kBlueStart+1) % 255]

 For a 16 bit gray image,

 - (0, 0): kGrayStart
 - (0, 1): (kGrayStart+1) % 65535

 This enables testing below to always know what the first pixel value should be
 at coordinate (0, 0).  The tests for image load verification rely on the
 pattern created in the constructor here, changing the pattern requires
 updating the tests as well.

 Tip: if a test image is desired to be viewed, run tests with --sandbox_debug
 so that the temporary files are not deleted.  Note that `no_cleanup` needs to
 be set to `true` for a test case, and each test usually removes the image
 with `fs::remove` so these will need to be updated to keep the image. */
template <int channels, int bit_depth>
class TestPng {
 public:
  static_assert(channels == 1 || channels == 3 || channels == 4,
                "TestPng <channels> must be 1, 3, or 4");
  static_assert(bit_depth == 8 || bit_depth == 16,
                "TestPng <bit_depth> must be 8 or 16");
  static_assert((channels == 1 && bit_depth == 16) ||      // 16 bit gray
                    (channels == 3 && bit_depth == 8) ||   // 8 bit RGB
                    (channels == 4 && bit_depth == 8) ||   // 8 bit RGBA
                    (channels == 3 && bit_depth == 16) ||  // 16 bit RGB
                    (channels == 1 && bit_depth == 8),     // 8 bit gray
                "TestPng: unsupported <channels, bit_depth> combination.");

  // Data type to allocate for memory, either uint8_t or uint16_t.
  using ChannelType = typename TestPngTraits<bit_depth>::ChannelType;
  static_assert(
      std::is_same_v<ChannelType, uint8_t> ||
          std::is_same_v<ChannelType, uint16_t>,
      "TestPng: only uint8_t and uint16_t channel types are supported.");

  // For when writing a color image.
  static constexpr ChannelType kRedStart = static_cast<ChannelType>(50);
  static constexpr ChannelType kGreenStart = static_cast<ChannelType>(100);
  static constexpr ChannelType kBlueStart = static_cast<ChannelType>(150);
  static constexpr ChannelType kAlphaStart = static_cast<ChannelType>(0);

  // For testing against RGB, alpha should be padded to 255.
  static constexpr ChannelType kAlphaPad = static_cast<ChannelType>(255);

  // For when writing a single channel image.
  static constexpr ChannelType kGrayStart = static_cast<ChannelType>(0);

  /* Creates a PNG image at the specified path with dimensions width x height,
   generating the pattern described on the class documentation.  User is
   responsible for deleting the file after it is created. */
  TestPng(const std::string& path, int width, int height);

  /* Returns `current` value, but also increments `current` by 1.  If increasing
   by 1 would put `current` out of bounds for the numeric limits of ChannelType,
   current will be reset to 0.

   Convenience method for loop bodies so that the increase and bounds check do
   not need to be repeated in every case (e.g., red, green, blue, alpha). */
  static ChannelType TestPngValue(ChannelType* current);

  /* If the image coordinates are wrong for loading, this test will fail with
   actual test information being reported in the failure.  Only to be used for
   channels == 3 (RGB) or channels == 4 (RGBA). */
  static void CornerCheckColor(const systems::sensors::ImageRgba8U& image);

  /* Tests all the pixels in the test image, but error reporting is not helpful
   in identifying what was wrong where.  Only to be used for channels == 3
   (RGB) or channels == 4 (RGBA).*/
  static void FullImageCheckColor(const systems::sensors::ImageRgba8U& image);

  /* If the image coordinates are wrong for loading, this test will fail with
   actual test information being reported in the failure.  Only to be used for
   channels == 1 (gray). */
  template <class DrakeImage>
  static void CornerCheckGray(const DrakeImage& image) {
    // TODO(svenevs): support for ImageDepth32f can be added.
    static_assert(std::is_same_v<DrakeImage, systems::sensors::ImageLabel16I>,
                  "Unexpected drake image type for TestPng::CornerCheckGray.");
    using DestType = typename DrakeImage::T;
    // Safeguard against incorrect usage of this function with non gray types.
    EXPECT_EQ(channels, 1);

    EXPECT_EQ(image.at(0, 0)[0], static_cast<DestType>(kGrayStart));
  }

  /* Tests all the pixels in the test image, but error reporting is not helpful
   in identifying what was wrong where. */
  template <class DrakeImage>
  static void FullImageCheckGray(const DrakeImage& image) {
    // TODO(svenevs): support for ImageDepth32f can be added.
    static_assert(std::is_same_v<DrakeImage, systems::sensors::ImageLabel16I>,
                  "Unexpected drake image type for TestPng::CornerCheckGray.");
    using DestType = typename DrakeImage::T;
    // Safeguard against incorrect usage of this function with non gray types.
    EXPECT_EQ(channels, 1);

    int n_good{0};
    const int n_expected = image.width() * image.height();
    ChannelType gray{kGrayStart};
    for (int j = 0; j < image.height(); ++j) {
      for (int i = 0; i < image.width(); ++i) {
        n_good += static_cast<int>(image.at(i, j)[0] ==
                                   static_cast<DestType>(TestPngValue(&gray)));
      }
    }
    EXPECT_EQ(n_good, n_expected);
  }

  const std::string path_;
  const int width_;
  const int height_;
};

// Instantiate them once in the source file.
extern template class TestPng<3, 8>;   // TestPngRgb8
extern template class TestPng<4, 8>;   // TestPngRgba8
extern template class TestPng<1, 16>;  // TestPngGray16
/* NOTE: these do not generate the same expected patterns, these are only used
 to generate a file to check that loading with the wrong bit width / channel
 type raise an error. */
extern template class TestPng<3, 16>;  // TestPngRgb16
extern template class TestPng<1, 8>;   // TestPngGray8

// Declare nice type names for test cases to use.
using TestPngRgb8 = TestPng<3, 8>;
using TestPngRgba8 = TestPng<4, 8>;
using TestPngGray16 = TestPng<1, 16>;
using TestPngRgb16 = TestPng<3, 16>;
using TestPngGray8 = TestPng<1, 8>;

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake

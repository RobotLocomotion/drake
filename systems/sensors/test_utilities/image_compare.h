#pragma once

#include <filesystem>
#include <ostream>

#include <gtest/gtest.h>

#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

/** Adds googletest support for printing EXPECT_EQ(Image, Image) failures.
Small images print all pixel data. Large images only print a summary. */
template <PixelType kPixelType>
void PrintTo(const Image<kPixelType>& image, std::ostream* os);

/** Loads the PNG or TIFF image from `filename` into the `image` output. */
template <PixelType kPixelType>
::testing::AssertionResult LoadImage(const std::filesystem::path& filename,
                                     Image<kPixelType>* image);

/** Saves the PNG or TIFF image to `$TEST_UNDECLARED_OUTPUTS_DIR/{filename}`.
TODO(jwnimmer-tri): the following pixel types are *NOT* supported:
- SaveUndeclaredOutputImage<PixelType::kRgb8U>
- SaveUndeclaredOutputImage<PixelType::kBgr8U>
- SaveUndeclaredOutputImage<PixelType::kBgra8U>
*/
template <PixelType kPixelType>
void SaveUndeclaredOutputImage(const Image<kPixelType>& image,
                               const std::filesystem::path& filename);

}  // namespace sensors
}  // namespace systems
}  // namespace drake

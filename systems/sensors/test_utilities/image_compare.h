#pragma once

#include <filesystem>

#include <fmt/ostream.h>
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

// TODO(jwnimmer-tri) Add a helper function to stash an image into the
// $TEST_UNDECLARED_OUTPUTS_DIR for offline inspection.

}  // namespace sensors
}  // namespace systems
}  // namespace drake

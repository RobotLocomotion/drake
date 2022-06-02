#pragma once

#include <string>

#include "drake/common/find_resource.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

template <typename T>
void CreateZeroDrakeImage(int width, int height, T* drake_image) {
  drake_image->resize(width, height);
  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      for (int c = 0; c < drake_image->kNumChannels; ++c) {
        drake_image->at(i, j)[c] = 0;
      }
    }
  }
}

/* Compares whether two drake::systems::sensors::Image are identical by
 iterating through each pixel value. */
template <typename T>
bool CompareSensorImages(const T& output_image, const T& expected_image) {
  if (output_image.width() != expected_image.width() ||
      output_image.height() != expected_image.height() ||
      output_image.kNumChannels != expected_image.kNumChannels) {
    return false;
  }

  for (int j = 0; j < expected_image.height(); ++j) {
    for (int i = 0; i < expected_image.width(); ++i) {
      for (int c = 0; c < expected_image.kNumChannels; ++c) {
        if (output_image.at(i, j)[c] != expected_image.at(i, j)[c])
          return false;
      }
    }
  }
  return true;
}

/* Recreates an ImageRgba8U image that should be exactly the same as the loaded
 test_{rgb, rgba}_8U.png.

 Returns an image (width=3, height=2) with the RGB values uniformly spaced
 between 0 and 255, i.e., 0, 51, 102, 153, 204, and 255, in the row-major order.
 The alpha channel will be padded to 255 if `num_channel` equals to 3 or the
 same as RGB channels otherwise. */
systems::sensors::ImageRgba8U CreateTestColorImage(int num_channel = 4);

/* Recreates an ImageDepth32F image that should be exactly the same as the
 loaded test_depth_32F.tiff.

 Returns an image (width=3, height=2) with the depth values uniformly spaced
 between 0.0 and 5.0, i.e., 0.0, 1.0, 2.0, 3.0, 4.0, and 5.0, in the row-major
 order. */
systems::sensors::ImageDepth32F CreateTestDepthImage();

/* Recreates an ImageLabel16I image that should be exactly the same as the
 loaded test_depth_16I.png.

 Returns an image (width=3, height=2) with the label values uniformly spaced
 between 0 and 5, i.e., 0, 1, 2, 3, 4, and 5, in the row-major order. */
systems::sensors::ImageLabel16I CreateTestLabelImage();

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake

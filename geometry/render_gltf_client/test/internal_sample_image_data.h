#pragma once

#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

/* Helper functions to validate the image loading functionality of
 RenderClient::Load{Color, Depth, Label}Image(). The pixel value assignments are
 kept explicit intentionally to enhance clarity. */

/* Recreates an ImageRgba8U image that should be exactly the same
 as the loaded test_{rgb, rgba}_8U.png. `pad_alpha` is set to true for RGB image
 comparison.

     ┌─────────────────┬─────────────────┬─────────────────┐
     │     0,1,2,3     │   10,11,12,13   │   20,21,22,23   │
     ├─────────────────┼─────────────────┼─────────────────┤
     │   30,31,32,33   │   40,41,42,43   │   50,51,52,53   │
     └─────────────────┴─────────────────┴─────────────────┘

 Returns an image (width=3, height=2) where every R, G, B, & A value is unique.
 If `pad_alpha` is true, the alpha value shown above is used. Otherwise, alpha
 is 255 for all pixels. */
systems::sensors::ImageRgba8U CreateTestColorImage(bool pad_alpha);

/* Recreates an ImageDepth32F image that should be exactly the same as the
 loaded test_depth_32F.tiff.

     ┌─────────────────┬─────────────────┬─────────────────┐
     │       0.0       │       1.0       │       2.0       │
     ├─────────────────┼─────────────────┼─────────────────┤
     │       3.0       │       4.0       │       5.0       │
     └─────────────────┴─────────────────┴─────────────────┘

 Returns an image (width=3, height=2) where every depth value is unique. */
systems::sensors::ImageDepth32F CreateTestDepthImage();

/* Recreates an ImageLabel16I image that should be exactly the same as the
 loaded test_colored_label_rgba_8U.png after it has been converted. Note that
 the value of the sixth pixel is set to render::RenderLabel::kDontCare to test
 the special conversion of white color (see render_gltf_client_doxygen.h).

     ┌─────────────────┬─────────────────┬─────────────────┐
     │        0        │        1        │        2        │
     ├─────────────────┼─────────────────┼─────────────────┤
     │        3        │        4        │    kDontCare    │
     └─────────────────┴─────────────────┴─────────────────┘

 Returns an image (width=3, height=2) where every label value is unique. */
systems::sensors::ImageLabel16I CreateTestLabelImage();

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake

#include "drake/systems/sensors/rgbd_renderer_ospray.h"
#include "drake/systems/sensors/test/rgbd_renderer_test_util.h"

namespace drake {
namespace systems {
namespace sensors {
namespace test {

using RgbdRendererOSPRayTest = RgbdRendererTest<RgbdRendererOSPRay>;
using Eigen::Isometry3d;


TEST_F(RgbdRendererOSPRayTest, CylinderTest) {
  Init(X_WC_, false);

  // Sets up a cylinder.
  Isometry3d X_WV = Isometry3d::Identity();
  X_WV.translation().z() = 0.6;
  DrakeShapes::VisualElement visual(X_WV);
  visual.setGeometry(DrakeShapes::Cylinder(0.2, 1.2));  // Radius and length.
  const int kBodyID = 1;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);
  renderer_->RenderColorImage(&color_);

  // Verifies outside the cylinder.
  for (const auto& p : kOutliers) {
    CompareColor(color_.at(p.x, p.y), ColorI({0, 0, 0}), 0,
                 kColorPixelTolerance);
  }
  // Verifies inside the cylinder.
  const int x = kInlier.x;
  const int y = kInlier.y;
  CompareColor(color_.at(x, y), ColorI({255, 255, 255}), 254u,
               kColorPixelTolerance);
}

}  // namespace test
}  // namespace sensors
}  // namespace systems
}  // namespace drake

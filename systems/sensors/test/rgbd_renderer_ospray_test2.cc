#include "drake/systems/sensors/rgbd_renderer_ospray.h"
#include "drake/systems/sensors/test/rgbd_renderer_test_util.h"

namespace drake {
namespace systems {
namespace sensors {
namespace test {

using RgbdRendererOSPRayTest = RgbdRendererTest<RgbdRendererOSPRay>;
using Eigen::Isometry3d;


TEST_F(RgbdRendererOSPRayTest, BoxTest) {
  Init(X_WC_, false);

  // Sets up a box.
  Isometry3d X_WV = Isometry3d::Identity();
  X_WV.translation().z() = 0.5;
  DrakeShapes::VisualElement visual(X_WV);
  Eigen::Vector3d box_size(1, 1, 1);
  visual.setGeometry(DrakeShapes::Box(box_size));
  const int kBodyID = 0;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);
  renderer_->RenderColorImage(&color_);

  // Verifies outside the box.
  for (const auto& p : kOutliers) {
    CompareColor(color_.at(p.x, p.y), ColorI({0, 0, 0}), 0,
                 kColorPixelTolerance);
  }
  // Verifies inside the box.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  CompareColor(color_.at(x, y), ColorI({255, 255, 255}), 254u,
               kColorPixelTolerance);
}

}  // namespace test
}  // namespace sensors
}  // namespace systems
}  // namespace drake

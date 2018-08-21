#include "drake/systems/sensors/rgbd_renderer_ospray.h"
#include "drake/systems/sensors/test/rgbd_renderer_test_util.h"

namespace drake {
namespace systems {
namespace sensors {
namespace test {

using RgbdRendererOSPRayTest = RgbdRendererTest<RgbdRendererOSPRay>;
using Eigen::Isometry3d;

TEST_F(RgbdRendererOSPRayTest, MeshTest) {
  Init(X_WC_, false);

  Isometry3d X_WV = Isometry3d::Identity();
  DrakeShapes::VisualElement visual(X_WV);
  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  visual.setGeometry(DrakeShapes::Mesh("", filename));
  const int kBodyID = 0;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);
  renderer_->RenderColorImage(&color_);
  // Verifies outside the mesh.
  for (const auto& p : kOutliers) {
    CompareColor(color_.at(p.x, p.y), ColorI({0, 0, 0}), 0,
                 kColorPixelTolerance);
  }
  // Verifies inside the mesh.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  CompareColor(color_.at(x, y), ColorI({104, 255, 129}), 214u, 2);
}

}  // namespace test
}  // namespace sensors
}  // namespace systems
}  // namespace drake

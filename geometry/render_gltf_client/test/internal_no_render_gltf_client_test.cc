#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render_gltf_client/factory.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace {

GTEST_TEST(RenderEngineGltfClientDisabledTest, ExceptionMessage) {
  const RenderEngineGltfClientParams params;
  DRAKE_EXPECT_THROWS_MESSAGE(MakeRenderEngineGltfClient(params),
                              "RenderEngineGltfClient was not compiled.*");
}

}  // namespace
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake

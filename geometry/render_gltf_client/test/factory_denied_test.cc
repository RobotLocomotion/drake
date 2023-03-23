#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render_gltf_client/factory.h"

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(FactoryDeniedTest, ExceptionMessage) {
  const RenderEngineGltfClientParams params;
  DRAKE_EXPECT_THROWS_MESSAGE(MakeRenderEngineGltfClient(params),
                              ".*DRAKE_ALLOW_NETWORK.*");
}

}  // namespace
}  // namespace geometry
}  // namespace drake

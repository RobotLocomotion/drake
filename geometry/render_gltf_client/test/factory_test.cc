#include "drake/geometry/render_gltf_client/factory.h"

#include <gtest/gtest.h>

#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render_gltf_client/internal_render_engine_gltf_client.h"
#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"

namespace drake {
namespace geometry {
namespace {

using geometry::render::RenderEngine;
using geometry::render_gltf_client::internal::RenderEngineGltfClient;

GTEST_TEST(FactoryTest, MakeRenderEngineGltfClient) {
  const std::unique_ptr<RenderEngine> render_engine =
      MakeRenderEngineGltfClient(RenderEngineGltfClientParams{});
  const RenderEngineGltfClient* render_engine_gltf_client =
      dynamic_cast<RenderEngineGltfClient*>(render_engine.get());

  EXPECT_NE(render_engine_gltf_client, nullptr);
  // Check the default RenderEngineGltfClientParams is propagated properly.
  EXPECT_EQ(render_engine_gltf_client->get_params().GetUrl(),
            "http://127.0.0.1:8000/render");
  EXPECT_EQ(render_engine_gltf_client->get_params().verbose, false);
  EXPECT_EQ(render_engine_gltf_client->get_params().cleanup, true);
}

}  // namespace
}  // namespace geometry
}  // namespace drake

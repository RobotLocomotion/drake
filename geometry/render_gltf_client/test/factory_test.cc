#include "drake/geometry/render_gltf_client/factory.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render_gltf_client/internal_render_engine_gltf_client.h"
#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"

namespace drake {
namespace geometry {
namespace {

using render::RenderEngine;
using render_gltf_client::internal::RenderEngineGltfClient;

/* Check whether the provided RenderEngineGltfClientParams is propagated
 correctly during the RenderEngineGltfClient construction. */
GTEST_TEST(FactoryTest, MakeRenderEngineGltfClient) {
  const RenderEngineGltfClientParams params{.base_url = "http://127.0.0.1:1234",
                                            .render_endpoint = "testing",
                                            .verbose = true,
                                            .cleanup = false};

  const std::unique_ptr<RenderEngine> render_engine =
      MakeRenderEngineGltfClient(params);
  const RenderEngineGltfClient* render_engine_gltf_client =
      dynamic_cast<RenderEngineGltfClient*>(render_engine.get());

  EXPECT_NE(render_engine_gltf_client, nullptr);
  EXPECT_EQ(render_engine_gltf_client->get_params().GetUrl(),
            params.base_url + "/" + params.render_endpoint);
  EXPECT_EQ(render_engine_gltf_client->get_params().verbose, params.verbose);
  EXPECT_EQ(render_engine_gltf_client->get_params().cleanup, params.cleanup);
}

}  // namespace
}  // namespace geometry
}  // namespace drake

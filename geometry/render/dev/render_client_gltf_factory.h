#pragma once

#include <memory>
#include <optional>
#include <string>

#include "drake/geometry/render/dev/render_client.h"

namespace drake {
namespace geometry {
namespace render {

struct RenderClientGLTFParams {
  std::optional<RenderLabel> default_label{};
  std::string url{"http://127.0.0.1"};
  unsigned port{8000};
  std::string upload_endpoint{"upload"};
  std::string render_endpoint{"render"};
  bool verbose = false;
  bool no_cleanup = false;
};

std::unique_ptr<RenderEngine> MakeRenderClientGLTF(
    const RenderClientGLTFParams& params);

}  // namespace render
}  // namespace geometry
}  // namespace drake

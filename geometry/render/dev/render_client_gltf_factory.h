#pragma once

#include <memory>
#include <optional>
#include <string>

#include "drake/geometry/render/dev/render_client.h"

namespace drake {
namespace geometry {
namespace render {

// TODO(svenevs): is there a way to avoid duplicating these from RenderClient?
struct RenderClientGLTFParams {
  std::optional<RenderLabel> default_label{};
  std::string url{"http://127.0.0.1"};
  unsigned port{8000};
  std::string upload_endpoint{"upload"};
  std::string render_endpoint{"render"};
  bool curl_verbose = true;

  // TODO(svenevs): shadow parameters from VTK renderer.  Expose?
  std::optional<Eigen::Vector4d> default_diffuse{};
  Eigen::Vector3d default_clear_color{204 / 255., 229 / 255., 255 / 255.};
};

std::unique_ptr<RenderEngine> MakeRenderClientGLTF(
    const RenderClientGLTFParams& params);

}  // namespace render
}  // namespace geometry
}  // namespace drake

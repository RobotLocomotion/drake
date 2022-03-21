#pragma once

#include <memory>
#include <optional>
#include <string>

#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {
namespace render {

/** Construction parameters for the RenderEngineGltfClient. */
struct RenderEngineGltfClientParams {
  /** The (optional) label to apply when none is otherwise specified.  */
  std::optional<RenderLabel> default_label{};

  /** The RenderClient::url() to use communicate with. */
  std::string url{"http://127.0.0.1"};

  /** The RenderClient::port() to communicate on.  A value less than or equal to
   `0` implies no port level communication is needed. */
  int port{8000};

  /** The RenderClient::render_endpoint() to retrieve renderings from. */
  std::string render_endpoint{"render"};

  /** Whether or not the RenderEngineGltfClient should log information about
   which files are being generated.  @sa RenderClient::verbose() */
  bool verbose = false;

  /** Whether or not the RenderEngineGltfClient should cleanup files generated /
   retrieved from the server.  @sa RenderClient::no_cleanup() */
  bool no_cleanup = false;
};

/** Constructs a RenderEngine implementation which generates
 <a href="https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html">glTF
 </a> files to upload to a rendering server, and retrieves renderings from said
 server to copy back into drake systems::sensors::Image buffers. */
std::unique_ptr<RenderEngine> MakeRenderEngineGltfClient(
    const RenderEngineGltfClientParams& params);

}  // namespace render
}  // namespace geometry
}  // namespace drake

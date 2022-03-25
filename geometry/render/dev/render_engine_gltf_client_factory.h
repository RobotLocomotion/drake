#pragma once

#include <memory>
#include <optional>
#include <string>

#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {
namespace render {

/** Construction parameters for the MakeRenderEngineGltfClient() to create a
 client as part of the @ref gltf_server_api. */
struct RenderEngineGltfClientParams {
  /** The (optional) label to apply when none is otherwise specified.  */
  std::optional<RenderLabel> default_label{};

  /** The url of the server communicate with.  Should **not** include a trailing
   `/` character.  For example, `https://drake.mit.edu` is acceptable while
   `https://drake.mit.edu/` is not. */
  std::string url{"http://127.0.0.1"};

  /** The port to communicate on.  A value less than or equal to `0` implies no
   port level communication is needed. */
  int port{8000};

  /** The server endpoint to retrieve renderings from.  Communications will be
   formed as `{url}/{render_endpoint}`, if the server expects forms posted to
   `/` then this value should be the empty string. */
  std::string render_endpoint{"render"};

  /** Whether or not the client should log information about which files are
   being generated, information about HTTP communications being coordinated with
   the server.  Information is logged at the debug level, so your application
   will need to drake::set_log_level() to `"debug"`.  @sa drake::log() */
  bool verbose = false;

  /** Whether or not the client should cleanup files generated / retrieved from
   the server.  By default (`no_cleanup=false`), as soon as a glTF scene file
   as well as server image response for a given frame is no longer needed they
   will be deleted.  To inspect generated scene files or server response images
   set `no_cleanup=true` to prevent the files and their governing temporary
   directory from being deleted.  During the construction process a number of
   copies and clones are created, when `no_cleanup=true` there will be more than
   one empty temporary directory created that will not be deleted.  The path
   to the temporary directory can be observed by setting
   RenderEngineGltfClientParams::verbose to `true`, or inspecting the parent
   directory described by drake::temp_directory(). */
  bool no_cleanup = false;
};

/** Constructs a RenderEngine implementation which generates
 <a href="https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html">glTF
 </a> files to upload to a rendering server, and retrieves renderings from said
 server to copy back into drake systems::sensors::Image buffers.  The server
 location, port, and endpoint are provided through the specified
 RenderEngineGltfClientParams.  The returned RenderEngine implements the client
 side of the @ref gltf_server_api.  The rules for supported geometries and
 textures are described in MakeRenderEngineVtk(). */
std::unique_ptr<RenderEngine> MakeRenderEngineGltfClient(
    const RenderEngineGltfClientParams& params);

}  // namespace render
}  // namespace geometry
}  // namespace drake

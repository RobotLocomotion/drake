#pragma once

#include <optional>
#include <string>

#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {

/** Construction parameters for the MakeRenderEngineGltfClient() to create a
 client as part of the @ref render_engine_gltf_client_server_api. */
struct RenderEngineGltfClientParams {
  /** The (optional) label to apply when none is otherwise specified.  */
  std::optional<render::RenderLabel> default_label{};

  /** The base url of the server communicate with.  Any trailing slashes will be
   pruned when querying the full url from GetUrl() method. */
  std::string base_url{"http://127.0.0.1"};

  /** The port to communicate on.  A value less than or equal to `0` will let
   `base_url` to decide which port to use.  If a different port is needed
   instead, specify `port` to override that. */
  int port{8000};

  /** The server endpoint to retrieve renderings from.  Any leading slashes will
   be pruned when querying the full url from GetUrl() method.  Trailing slashes
   , however, will be kept as-is.  If the server expects forms posted to `/`
   then this value should be the empty string. */
  std::string render_endpoint{"render"};

  /** Whether or not the client should log information about which files are
   being generated, as well as any information about HTTP communications between
   the client and server such as HTTP header information, url and port, etc.
   Information is logged at the debug level, so your application will need to
   drake::set_log_level() to `"debug"`.  @sa drake/common/text_logging.h */
  bool verbose = false;

  /** Whether or not the client should cleanup files generated / retrieved from
   the server.  By default (`no_cleanup=false`), as soon as a glTF scene file
   as well as server image response for a given frame are no longer needed they
   will be deleted.  To inspect generated scene files or server response images
   set `no_cleanup=true` to prevent the files and their governing temporary
   directory from being deleted.  During the construction process a number of
   copies and clones are created, when `no_cleanup=true` there will be more than
   one empty temporary directory created that will not be deleted.  The path
   to the temporary directory can be observed by setting
   RenderEngineGltfClientParams::verbose to `true`, or inspecting the parent
   directory described by drake::temp_directory(). */
  bool no_cleanup = false;

  /** Returns the post-processed full URL used for client-server communication.
   The full url is constructed as `{url}/{endpoint}` where any trailing slashes
   in `base_url` and any leading slashes in `render_endpoint` are removed.

   Throws an exception if either `base_url` or `render_endpoint` becomes the
   empty string after slash pruning. */
  std::string GetUrl() const;
};

}  // namespace geometry
}  // namespace drake

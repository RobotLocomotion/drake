#pragma once

#include <memory>
#include <optional>
#include <string>

#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {

/** Construction parameters for the MakeRenderEngineGltfClient() to create a
 client as part of the @ref render_engine_gltf_client_server_api. */
struct RenderEngineGltfClientParams {
  /** The (optional) label to apply when none is otherwise specified.  */
  std::optional<render::RenderLabel> default_label{};

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
   being generated, as well as any information about HTTP communications between
   the client and server such as HTTP header information, url and port, etc.
   Information is logged at the debug level, so your application will need to
   drake::set_log_level() to `"debug"`.  @sa drake::log() */
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
};

/** Constructs a RenderEngine implementation which generates
 <a href="https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html">glTF
 </a> files to upload to a rendering server, and retrieves renderings from said
 server to copy back into drake systems::sensors::Image buffers.  The server
 url, port, and endpoint are provided through the specified
 RenderEngineGltfClientParams.  The returned RenderEngine implements the client
 side of the @ref render_engine_gltf_client_server_api.  The rules for supported
 geometries and textures are described in MakeRenderEngineVtk().

 @note
   The underlying RenderEngine utilizes [libcurl][libcurl] to communicate with
   the server.  Static curl initialization must be performed once per process,
   and this operation is [**not** thread-safe][libcurl_threadsafe]!  Curl is
   initialized by instantiating this RenderEngine with the default
   [`curl_global_init(CURL_GLOBAL_ALL | CURL_GLOBAL_ACK_EINTR)`][libcurl_init],
   the implication for consuming applications being:
 @note
   1. If you intend to have your rendering take place in a threaded context,
      you **must** instantiate this RenderEngine via %MakeRenderEngineGltfClient
      from the main thread **before** spawning your threaded workers.  As soon
      as one of these RenderEngine instances has been constructed, libcurl will
      have been initialized.
      @code{.cpp}
      // Setup your server information and create the RenderEngine.  This must
      // be done in a non-threaded context (e.g., at program start).
      RenderEngineGltfClientParams params;
      params.url = "http://some-server.url";
      auto render_engine = MakeRenderEngineGltfClient(params);

      // After MakeRenderEngineGltfClient, libcurl has been initialized and you
      // may now create whatever threads desired.
      @endcode
   2. If you need to use a different initialization strategy for libcurl in your
      application, you should first create the RenderEngine using
      %MakeRenderEngineGltfClient, then manually call
      [`curl_global_cleanup()`][libcurl_cleanup], followed by manually
      calling [`curl_global_init(...)`][libcurl_init] with your desired flags.
      Generally speaking, this scenario is **atypical** and you should not need
      to worry about this.  Applications with specialized libcurl needs, though,
      must understand the construction and initialization order to be able to
      modify the behavior to suit their needs.
      @code{.cpp}
      // Setup your server information and create the RenderEngine.  This must
      // be done in a non-threaded context (e.g., at program start).
      RenderEngineGltfClientParams params;
      params.url = "http://some-server.url";
      auto render_engine = MakeRenderEngineGltfClient(params);

      // After MakeRenderEngineGltfClient, libcurl has been initialized so your
      // application needs to cleanup and re-initialize as needed.
      curl_global_cleanup();
      curl_global_init();  // <<< your custom flags here

      // Now that libcurl has been re-initialized to suit your application's
      // needs, you may now create whatever threads desired.
      @endcode

 [libcurl]: https://curl.se/libcurl/
 [libcurl_threadsafe]: https://curl.se/libcurl/c/threadsafe.html
 [libcurl_init]: https://curl.se/libcurl/c/curl_global_init.html
 [libcurl_cleanup]: https://curl.se/libcurl/c/curl_global_cleanup.html
 */
std::unique_ptr<render::RenderEngine> MakeRenderEngineGltfClient(
    const RenderEngineGltfClientParams& params);

}  // namespace geometry
}  // namespace drake

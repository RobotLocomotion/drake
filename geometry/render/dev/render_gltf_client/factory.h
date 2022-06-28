#pragma once

#include <memory>

#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"

namespace drake {
namespace geometry {

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
      params.base_url = "http://some-server.url";
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
      params.base_url = "http://some-server.url";
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

#pragma once

#include <memory>

#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"

namespace drake {
namespace geometry {

/** Reports the availability of the RenderEngineGltfClient implementation. */
extern const bool kHasRenderEngineGltfClient;

/** Constructs a RenderEngine implementation which generates
 <a href="https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html">glTF
 </a> files to upload to a render server, and retrieves renderings from said
 server by copying image data back into drake systems::sensors::Image buffers.
 The server url and endpoint are provided through the specified
 RenderEngineGltfClientParams.  The returned RenderEngine implements the client
 side of the @ref render_engine_gltf_client_server_api.  The rules for supported
 geometries and textures are the same as for the VTK-based engine and are
 described in MakeRenderEngineVtk().

 @note
   The underlying RenderEngine utilizes [libcurl][libcurl] to communicate with
   a server.  Static curl initialization must be performed once per process, and
   the operation is [**not** thread-safe][libcurl_threadsafe]!  Instantiating
   this RenderEngine automatically initializes curl with the default
   [`curl_global_init(CURL_GLOBAL_ALL | CURL_GLOBAL_ACK_EINTR)`][libcurl_init],
   the implication for consuming applications being:
 @note
   1. See \ref allow_network "DRAKE_ALLOW_NETWORK" for an environment variable
      option to deny remote rendering entirely.
   2. If you intend to have your rendering take place in a threaded context,
      you **must** instantiate this RenderEngine via %MakeRenderEngineGltfClient
      from the main thread **before** spawning your threaded workers.  As soon
      as one of these RenderEngine instances has been constructed, libcurl will
      have been initialized.
      @code{.cpp}
      // Setup your server information and create the RenderEngine.  This must
      // be done in a non-threaded context (e.g., at the program start).
      RenderEngineGltfClientParams params;
      params.base_url = "http://some-server.url";
      auto render_engine = MakeRenderEngineGltfClient(params);

      // After MakeRenderEngineGltfClient() function call, libcurl has been
      // initialized and you may now create threads if desired.
      @endcode
   3. If you need to use a different initialization strategy for libcurl in your
      application, you should first create the RenderEngine using
      %MakeRenderEngineGltfClient, then manually call
      [`curl_global_cleanup()`][libcurl_cleanup], followed by manually calling
      [`curl_global_init(...)`][libcurl_init] with your desired flags. In
      general, this scenario is **atypical** and you should not need to worry
      about this.  Applications with specialized libcurl needs, though, must
      understand the construction and initialization order to be able to modify
      the behavior to suit their needs.
      @code{.cpp}
      // Follow the steps above to construct the RenderEngine given the
      // specified RenderEngineGltfClientParams.

      // Libcurl has been initialized at this point, so your application needs
      // to reset and re-initialize curl again.
      curl_global_cleanup();
      curl_global_init(...);  // <<< your custom flags here

      // Now that libcurl has been re-initialized to suit your application's
      // needs, you may now create threads if desired.
      @endcode

 @throws std::exception if kHasRenderEngineGltfClient is false, or if disabled
 via the environment variable DRAKE_ALLOW_NETWORK.

 [libcurl]: https://curl.se/libcurl/
 [libcurl_threadsafe]: https://curl.se/libcurl/c/threadsafe.html
 [libcurl_init]: https://curl.se/libcurl/c/curl_global_init.html
 [libcurl_cleanup]: https://curl.se/libcurl/c/curl_global_cleanup.html
 */
std::unique_ptr<render::RenderEngine> MakeRenderEngineGltfClient(
    const RenderEngineGltfClientParams& params);

}  // namespace geometry
}  // namespace drake

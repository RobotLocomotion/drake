#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/geometry/render_gltf_client/factory.h"
// #include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"

// Symbol: pydrake_doc_geometry_render_gltf_client
constexpr struct /* pydrake_doc_geometry_render_gltf_client */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::geometry
    struct /* geometry */ {
      // Symbol: drake::geometry::MakeRenderEngineGltfClient
      struct /* MakeRenderEngineGltfClient */ {
        // Source: drake/geometry/render_gltf_client/factory.h
        const char* doc =
R"""(Constructs a RenderEngine implementation which generates <a
href="https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html">glTF
</a> files to upload to a render server, and retrieves renderings from
said server by copying image data back into drake
systems∷sensors∷Image buffers. The server url and endpoint are
provided through the specified RenderEngineGltfClientParams. The
returned RenderEngine implements the client side of the
render_engine_gltf_client_server_api. The rules for supported
geometries and textures are the same as for the VTK-based engine and
are described in MakeRenderEngineVtk().

Note:
    The underlying RenderEngine utilizes [libcurl][libcurl] to
    communicate with a server. Static curl initialization must be
    performed once per process, and the operation is [**not**
    thread-safe][libcurl_threadsafe]! Instantiating this RenderEngine
    automatically initializes curl with the default
    [`curl_global_init(CURL_GLOBAL_ALL |
    CURL_GLOBAL_ACK_EINTR)`][libcurl_init], the implication for
    consuming applications being:

Note:
1. See allow_network "DRAKE_ALLOW_NETWORK" for an environment variable
option to deny remote rendering entirely.
2. If you intend to have your rendering take place in a threaded context,
you **must** instantiate this RenderEngine via MakeRenderEngineGltfClient
from the main thread **before** spawning your threaded workers.  As soon
as one of these RenderEngine instances has been constructed, libcurl will
have been initialized.



.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // Setup your server information and create the RenderEngine.  This must
    // be done in a non-threaded context (e.g., at the program start).
    RenderEngineGltfClientParams params;
    params.base_url = "http://some-server.url";
    auto render_engine = MakeRenderEngineGltfClient(params);
    
    // After MakeRenderEngineGltfClient() function call, libcurl has been
    // initialized and you may now create threads if desired.

.. raw:: html

    </details>

3. If you need to use a different initialization strategy for libcurl in your
application, you should first create the RenderEngine using
MakeRenderEngineGltfClient, then manually call
[`curl_global_cleanup()`][libcurl_cleanup], followed by manually calling
[`curl_global_init(...)`][libcurl_init] with your desired flags. In
general, this scenario is **atypical** and you should not need to worry
about this.  Applications with specialized libcurl needs, though, must
understand the construction and initialization order to be able to modify
the behavior to suit their needs.



.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // Follow the steps above to construct the RenderEngine given the
    // specified RenderEngineGltfClientParams.
    
    // Libcurl has been initialized at this point, so your application needs
    // to reset and re-initialize curl again.
    curl_global_cleanup();
    curl_global_init(...);  // <<< your custom flags here
    
    // Now that libcurl has been re-initialized to suit your application's
    // needs, you may now create threads if desired.

.. raw:: html

    </details>

Raises:
    RuntimeError if kHasRenderEngineGltfClient is false, or if
    disabled via the environment variable DRAKE_ALLOW_NETWORK.

[libcurl]: https://curl.se/libcurl/ [libcurl_threadsafe]:
https://curl.se/libcurl/c/threadsafe.html [libcurl_init]:
https://curl.se/libcurl/c/curl_global_init.html [libcurl_cleanup]:
https://curl.se/libcurl/c/curl_global_cleanup.html)""";
      } MakeRenderEngineGltfClient;
      // Symbol: drake::geometry::RenderEngineGltfClientParams
      struct /* RenderEngineGltfClientParams */ {
        // Source: drake/geometry/render_gltf_client/render_engine_gltf_client_params.h
        const char* doc =
R"""(Construction parameters for the MakeRenderEngineGltfClient() to create
a client as part of the render_engine_gltf_client_server_api.)""";
        // Symbol: drake::geometry::RenderEngineGltfClientParams::GetUrl
        struct /* GetUrl */ {
          // Source: drake/geometry/render_gltf_client/render_engine_gltf_client_params.h
          const char* doc =
R"""(Returns the post-processed full url used for client-server
communication. The full url is constructed as
``{base_url}/{render_endpoint}`` where all trailing slashes in
``base_url`` and all leading slashes in ``render_endpoint`` have been
removed.)""";
        } GetUrl;
        // Symbol: drake::geometry::RenderEngineGltfClientParams::Serialize
        struct /* Serialize */ {
          // Source: drake/geometry/render_gltf_client/render_engine_gltf_client_params.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::geometry::RenderEngineGltfClientParams::base_url
        struct /* base_url */ {
          // Source: drake/geometry/render_gltf_client/render_engine_gltf_client_params.h
          const char* doc =
R"""(The base url of the server communicate with. See GetUrl() for details.)""";
        } base_url;
        // Symbol: drake::geometry::RenderEngineGltfClientParams::cleanup
        struct /* cleanup */ {
          // Source: drake/geometry/render_gltf_client/render_engine_gltf_client_params.h
          const char* doc =
R"""(Whether or not the client should cleanup files generated / retrieved
from the server. By default (``cleanup=true``), after a server image
response has been loaded into the client's memory, the glTF scene file
and images will be deleted. To keep the generated scene files or
server response images for inspection purposes, set ``cleanup=false``
instead. During the construction process a number of copies and clones
are created, when ``cleanup=false`` there will be more than one empty
temporary directory created that will not be deleted. The path to the
temporary directory can be observed by setting
RenderEngineGltfClientParams∷verbose to ``True``, or inspecting the
parent directory described by drake∷temp_directory().)""";
        } cleanup;
        // Symbol: drake::geometry::RenderEngineGltfClientParams::render_endpoint
        struct /* render_endpoint */ {
          // Source: drake/geometry/render_gltf_client/render_engine_gltf_client_params.h
          const char* doc =
R"""((Advanced) The server endpoint to retrieve renderings from. See
GetUrl() for details.)""";
        } render_endpoint;
        // Symbol: drake::geometry::RenderEngineGltfClientParams::verbose
        struct /* verbose */ {
          // Source: drake/geometry/render_gltf_client/render_engine_gltf_client_params.h
          const char* doc =
R"""(Whether or not the client should log information about which files are
being generated, as well as any information about HTTP communications
between the client and server such as HTTP header information, url and
port, etc. Information is logged at the debug level, so your
application will need to logging∷set_log_level() to ``"debug"``.

See also:
    drake/common/text_logging.h)""";
        } verbose;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("base_url", base_url.doc),
            std::make_pair("cleanup", cleanup.doc),
            std::make_pair("render_endpoint", render_endpoint.doc),
            std::make_pair("verbose", verbose.doc),
          };
        }
      } RenderEngineGltfClientParams;
    } geometry;
  } drake;
} pydrake_doc_geometry_render_gltf_client;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif

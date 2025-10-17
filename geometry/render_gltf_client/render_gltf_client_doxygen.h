/** @file
 Doxygen-only documentation for @ref render_engine_gltf_client_server_api. */

namespace drake {
namespace geometry {
namespace render_gltf_client {

/** @defgroup render_engine_gltf_client_server_api glTF Render Client-Server API
    @ingroup render_engines

## Overview {#overview}

Drake offers built-in renderers (RenderEngineVtk, RenderEngineGl), but in some
cases users may want to use their own custom rendering implementations.  One way
to accomplish that is to subclass RenderEngine with a custom implementation and
link that into Drake, but sometimes that leads to linker compatibility problems.
Thus, we also offer another option: rendering using a remote procedure call
(RPC). This document specifies the network API for servicing those requests.

The [glTF][glTF] render server API consists of two components: a client, and a
server. The client generates and transmits glTF scene files to the server, which
then renders and returns an image file back to the client.  Drake implements the
client side, which can be constructed via MakeRenderEngineGltfClient().  The
server must respond with images of a certain type and dimension (width and
height), but is otherwise free to produce any rendering desired in however much
time it needs to render.

The primary design goal for this RPC architecture is for batch-mode dataset
generation (e.g., for machine learning), not for realtime simulation.
We transmit the entire scene its entirety for each RPC request; the protocol
does not support incremental changes to the scene.

The server is welcome to add in fixed scene elements beyond what's transmitted
(e.g., the server could add in a building's walls and floor, rather than having
the client transmit that same immutable geometry in every RPC).  However, for
anything dynamic the design intent is that every RPC call is comprehensive on
its own.

[glTF]: https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html

<b>Contents</b>:

- [Overview](#overview)
- [Server API](#server-api)
    - [Render Endpoint](#render-endpoint)
    - [Render Endpoint `<form>` Data](#render-endpoint-form-data)
    - [Allowed Image Response Types](#allowed-image-response-types)
    - [Notes on glTF Camera Specification](#notes-on-gltf-camera-specification)
    - [Notes on Communicating Errors](#notes-on-communicating-errors)
- [Notes on Rendering Label Images](#notes-on-rendering-label-images)
- [Existing Server Implementations](#existing-server-implementations)
- [Developing your own Server](#developing-your-own-server)

## Server API {#server-api}
<hr>

A given server implementation is required to implement a "Render Endpoint" to
which the client will issue a [`POST` request][html_post] with an
[html `<form>`][html_form] and then await a rendered image response.  The `POST`
from the client will include uploading the scene file, in addition to a variety
of other metadata attributes related to the image being rendered (such as
`width` and `height`), as well as the full specification of the
systems::sensors::CameraInfo intrinsics being rendered.  The scene file must
use the glTF 2.0 file format and must only use embedded assets (with `data:`
URIs). A future version of this protocol might allow for non-embedded assets.

The server is **expected to block** (delay sending a response) until it is ready
to transmit the final rendered image back to the client.  This provides for an
easier implementation of the server, as well as this single endpoint is the only
form of communication the client will initiate with the server.  If the server
fails to respond with a valid image file, the client will produce an error.

The server is **required** to send a valid [HTTP response code][http_responses]
in **both the event of success and failure**.  If an HTTP response code
indicating success (2XX and 3XX) is received, the client will further require an
image response of a supported image type with the correct dimensions.
Otherwise, the client will attempt to populate an error message from the file
response (if available).  As a special note about `flask` in particular, if the
server implementation doesn't return any code, the default response is always
`200` which indicates success.  When in doubt, have the server respond with
`200` for a success, `400` for any failure related to bad requests (e.g.,
missing `min_depth` or `max_depth` for an `image_type="depth"`), and `500` for
any unhandled errors.

[html_form]: https://developer.mozilla.org/en-US/docs/Web/HTML/Element/form
[html_post]: https://developer.mozilla.org/en-US/docs/Web/HTTP/Methods/POST
[http_responses]: https://developer.mozilla.org/en-US/docs/Web/HTTP/Status

### Render Endpoint {#render-endpoint}
<hr>

The render endpoint (by default: `/render`) is responsible for receiving an
uploaded scene file, rendering the scene, and transmitting an image back to the
client.  In addition to the scene file, the render endpoint is provided with the
full specification of the systems::sensors::CameraInfo object, and optionally
the depth range of the systems::sensors::DepthRange object.

### Render Endpoint `<form>` Data {#render-endpoint-form-data}
<hr>

The client will `POST` a `<form>` with an `enctype=multipart/form-data` to the
server with the following field entries:

<table>
  <tr>
    <th>Field Name</th>
    <th>Field Description</th>
  </tr>
  <tr>
    <td><b>scene</b></td>
    <td>
      The scene file contents.  Sent as if from
      `<input type="file" name="scene">`.  glTF scenes will have a mime type of
      [`model/gltf+json`][gltf_mimetypes].
    </td>
  </tr>
  <tr>
    <td><b>scene_sha256</b></td>
    <td>
      The sha256 hash of the `scene` file being uploaded.  The server may use
      this entry to validate that the full file was successfully uploaded.  Sent
      as form data `<input type="text" name="scene_sha256">`.
    </td>
  </tr>
  <tr>
    <td><b>image_type</b></td>
    <td>
      The type of image being rendered.  Its value will be one of: `"color"`,
      `"depth"`, or `"label"`.  Sent as form data
      `<input type="text" name="image_type">`.
    </td>
  </tr>
  <tr>
    <td><b>min_depth</b></td>
    <td>
      The minimum depth range as specified by a depth sensor's
      DepthRange::min_depth().  **Only provided when `image_type="depth"`.**
      Sent as form data `<input type="number" name="min_depth">`.  Decimal
      value.
    </td>
  </tr>
  <tr>
    <td><b>max_depth</b></td>
    <td>
      The maximum depth range as specified by a depth sensor's
      DepthRange::max_depth().  **Only provided when `image_type="depth"`.**
      Sent as form data `<input type="number" name="max_depth">`.  Decimal
      value.
    </td>
  </tr>
  <tr>
    <td><b>width</b></td>
    <td>
      Width of the desired rendered image in pixels as specified by the
      systems::sensors::CameraInfo::width() being rendered, the server **must**
      respond with an image of the same width.  Sent as form data
      `<input type="number" name="width">`.  Integral value.
    </td>
  </tr>
  <tr>
    <td><b>height</b></td>
    <td>
      Height of the desired rendered image in pixels as specified by the
      systems::sensors::CameraInfo::height() being rendered, the server **must**
      respond with an image of the same height.  Sent as form data
      `<input type="number" name="height">`.  Integral value.
    </td>
  </tr>
  <tr>
    <td><b>near</b></td>
    <td>
      The near clipping plane of the camera as specified by the
      RenderCameraCore's ClippingRange::near() value.  Sent as
      form data `<input type="number" name="near">`.  Decimal value.
    </td>
  </tr>
  <tr>
    <td><b>far</b></td>
    <td>
      The far clipping plane of the camera as specified by the
      RenderCameraCore's ClippingRange::far() value.  Sent as form data
      `<input type="number" name="far">`.  Decimal value.
    </td>
  </tr>
  <tr>
    <td><b>focal_x</b></td>
    <td>
      The focal length x, in pixels, as specified by the
      systems::sensors::CameraInfo::focal_x() value.  Sent as form data
      `<input type="number" name="focal_x">`.  Decimal value.
    </td>
  </tr>
  <tr>
    <td><b>focal_y</b></td>
    <td>
      The focal length y, in pixels, as specified by the
      systems::sensors::CameraInfo::focal_y() value.  Sent as form data
      `<input type="number" name="focal_y">`.  Decimal value.
    </td>
  </tr>
  <tr>
    <td><b>fov_x</b>
    <td>
      The field of view in the x-direction (in radians) as specified by the
      systems::sensors::CameraInfo::fov_x() value.  Sent as form data
      `<input type="number" name="fov_x">`.  Decimal value.
    </td>
  </tr>
  <tr>
    <td><b>fov_y</b>
    <td>
      The field of view in the y-direction (in radians) as specified by the
      systems::sensors::CameraInfo::fov_y() value.  Sent as form data
      `<input type="number" name="fov_y">`.  Decimal value.
    </td>
  </tr>
  <tr>
    <td><b>center_x</b></td>
    <td>
      The principal point's x coordinate in pixels as specified by the
      systems::sensors::CameraInfo::center_x() value.  Sent as form data
      `<input type="number" name="center_x">`.  Decimal value.
    </td>
  </tr>
  <tr>
    <td><b>center_y</b></td>
    <td>
      The principal point's y coordinate in pixels as specified by the
      systems::sensors::CameraInfo::center_y() value.  Sent as form data
      `<input type="number" name="center_y">`.  Decimal value.
    </td>
  </tr>
</table>

Note: {focal_x, focal_y} contains duplicated information as {fov_x, fov_y} given
an image size.  Nonetheless, both pieces of information are sent to the server
so that the server can choose its preferred representation.  The server can
expect them to be consistent.

[gltf_mimetypes]: https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#_media_type_registrations

### Allowed Image Response Types {#allowed-image-response-types}
<hr>

The client accepts the following image types from a server render:

- When `image_type="color"`, the server may return:
    - An RGB (3 channel) unsigned char PNG image.
    - An RGBA (4 channel) unsigned char PNG image.
- When `image_type="depth"`, the server may return:
    - A 32-bit float single channel TIFF image.  The client will interpret this
      rendering as units of meters.
    - A 16-bit integer single channel TIFF or PNG image.  The client will
      interpret this rendering as units of millimeters.  Pixels at their maximum
      value (2¹⁶-1) will be interpreted as kTooFar (i.e., infinity).
- When `image_type="label"`, the server may return:
    - An RGB (3 channel) unsigned char PNG image.
    - An RGBA (4 channel) unsigned char PNG image.
    - Note: The client will interpret this rendering as a colored label image
    and convert to the final label image. See
    [this section](#notes-on-rendering-label-images) for other requirements for
    returning a label image.

### Notes on glTF Camera Specification {#notes-on-gltf-camera-specification}
<hr>

For a [glTF scene][glTF] file, note that there are two locations that describe
the camera:

1. The [`"cameras"` array][glTF_cameras], which specifies the camera projection
   matrix.  The client will always produce a length one `"cameras"` array, with
   a single entry ("camera 0").  This camera will always be of
   `"type": "perspective"`, and its `"aspectRatio"`, `"yfov"`, `"zfar"`, and
   `"znear"` attributes will accurately represent the drake sensor.  However,
   note that the [glTF perspective projection definition][glTF_proj] does not
   include all of the information present in the matrix that would be obtained
   by `RenderCameraCore::CalcProjectionMatrix`.  While the two matrices will be
   similar, a given render server must decide based off its choice of render
   backend how it wishes to model the camera perspective projection
   transformation -- utilize the glTF definition, or incorporate the remainder
   of the `<form>` data to construct its own projection matrix.  A sample
   snippet from a client glTF file:

        {
          "cameras" :
          [
            {
              "perspective" :
              {
                "aspectRatio" : 1.3333333333333333,
                "yfov" : 0.78539816339744828,
                "zfar" : 10,
                "znear" : 0.01
             },
             "type" : "perspective"
            }
          ],
        }

2. The [`"nodes"` array][glTF_nodes], which specifies the camera's global
   transformation matrix.  The `"camera": 0` entry refers to the index into the
   `"cameras"` array from (1).  Note that this is **not** the "model view
   transformation" (rather, its inverse) -- it is the camera's global
   transformation matrix which places the camera in the world just like
   any other entry in `"nodes"`.  Note that the `"matrix"` is presented in
   column-major order, as prescribed by the glTF specification.  A sample
   snippet of the camera node specification in the `"nodes"` array that has
   no rotation (the identity rotation) and a translation vector
   `[x=0.1, y=0.2, z=0.3]` would be provided as:

        {
          "nodes" :
          [
            {
              "camera" : 0,
              "matrix" :
              [
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.1,
                0.2,
                0.3,
                1.0,
              ],
              "name" : "Camera Node"
            },
          ],
        }

[glTF_cameras]:https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#reference-camera
[glTF_nodes]: https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#reference-node
[glTF_proj]: https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#projection-matrices


### Notes on Communicating Errors {#notes-on-communicating-errors}
<hr>

When errors occur on the server side, the server should explicitly return an
HTTP response code indicating a failed transaction.

Additionally, the server should clearly communicate _why_ there was an upload or
render failure as plain text in the file response.  Though this is not strictly
required, the user of the server will have no hints as to what is going wrong
with the client-server communication.  When the file response is provided, this
information will be included in the exception message produced by the client.

## Notes on Rendering Label Images {#notes-on-rendering-label-images}
<hr>

Renderers typically can't render objects with "labels". Drake encodes the labels
associated with geometries as unique colors and provides those colors to the
server as attributes on the meshes. Thus, the label output from any server will
be an RGB or RGBA PNG.

__All renderable artifacts that exist _only_ in the server -- that are not part
of the Drake-provided glTF -- must be colored white (RGB=(255, 255, 255))__.
These server-only renderable artifacts include:

  - The background color.
  - Any geometries that the server loads (e.g., walls in a room, environment
    images, etc.).

When producing the final label output, the client will interpret this particular
RGB value as render::RenderLabel::kDontCare. This means that a remote server
will never report a pixel with the render::RenderLabel::kEmpty value.

For an image to be a proper color-encoded label image, the only pixel values in
the image must be one of the recognized label encodings. This may require
special render configurations. Any configurations that can introduce color
variation must be disabled. That includes (but is not limited to) the following
render features:

  - Anti-aliasing (multi-sampling)
  - Lighting
  - Color transformations on the rendered image

## Existing Server Implementations {#existing-server-implementations}
<hr>

[drake-blender] is a glTF render server using [Blender] as the backend.

[drake-blender]: https://github.com/RobotLocomotion/drake-blender
[Blender]: https://www.blender.org

## Developing your own Server {#developing-your-own-server}
<hr>

To test the basic client-server communication and rendering, Drake provides a
simple server implementation as a reference. For more information about
developing your own server or running the prototype, refer to the
[README][render_gltf_client_test] under `//geometry/render_gltf_client/test`.

[render_gltf_client_test]: https://github.com/RobotLocomotion/drake/blob/master/geometry/render_gltf_client/test

*/

}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake

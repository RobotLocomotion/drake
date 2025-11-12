/** @file
 Doxygen-only documentation for @ref render_engines.  */

namespace drake {
namespace geometry {
namespace render {

/** @addtogroup render_engines
 @{
 Rendering engines are used to enable simulation of the perception stack.

 <h2>Overview</h2>

 Perception simulation is supported via the RenderEngine API. It provides the
 interface through which the geometries in the simulation can be rendered to
 three types of images: color, depth, and label. (For more details about the
 API, refer to the RenderEngine documentation.)

 Drake includes *three* implementations of that API:

   - RenderEngineVtk - A GPU-based rasterization renderer using the VTK library.
   - RenderEngineGl - A GPU-based rasterization renderer using direct calls to
                      the OpenGL API.
   - RenderEngineGltfClient - An implementation of a client-server RPC renderer
                              that broadcasts Drake's visual state in a glTF
                              file.

 These implementations differ mostly in performance and flexibility.
 RenderEngineVtk is slower but has the possibility of leveraging the full VTK
 API to provide *post hoc* customizations.
 RenderEngineGl is faster, but gains that performance boost by implementing a
 barebones rendering pipeline. RenderEngineGltfClient can connect to a server
 backed by arbitrary rendering technology with the potential of producing the
 highest fidelity images possible, but with a much higher latency on producing
 individual images.

 Picking the right renderer for your simulated sensors will be based on
 considering those differences and picking the trade-off that best suits your
 application.

 The current API is simple and precludes sophisticated definitions of lights
 and materials. As the API matures, these additional features will be added.

 <h2>Instantiation</h2>

 A RenderEngine most typically serves as the core rendering technology for a
 "sensor" (e.g. systems::sensors::RgbdSensor,
 systems::sensors::RgbdSensorDiscrete, and systems::sensors::RgbdSensorAsync).
 While it's certainly possible to instantiate an engine and render directly,
 sensors are the *expected* mechanism to obtain rendered images.

 The simplest way to configure a sensor and its corresponding RenderEngine is
 with systems::sensors::CameraConfig and systems::sensors::ApplyCameraConfig().
 It can be done in code or via yaml. Any of the RenderEngine implementations
 can be selected and fully configured this way.

 See also:

   - geometry::RenderEngineGlParams and geometry::MakeRenderEngineGl().
   - geometry::RenderEngineVtkParams and geometry::MakeRenderEngineVtk().
   - geometry::RenderEngineGltfClientParams and
     geometry::MakeRenderEngineGltfClient().


 <h2>Performance</h2>

 The performance of both of the current implementations will strongly depend
 on the quality of a system's GPU and its driver. A more powerful graphics card
 with an up-to-date driver will likely produce images more quickly.

 It is impossible to state what the a RenderEngine's performance is in absolute
 terms. Instead, Drake includes a simple @ref render_engine_benchmarks
 "benchmark" which can be run locally to assess the actual performance the end
 user will experience on a fixed system.

 Next topic: @ref geometry_file_formats
 @}
 */

/** @defgroup render_benchmarks Render Benchmarks
 @ingroup render_engines

 Benchmarks for assessing rendering performance and comparing implementations.
 */

}  // namespace render
}  // namespace geometry
}  // namespace drake

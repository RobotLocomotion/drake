namespace drake {
namespace geometry {
namespace render {

/** @addtogroup render_engines

 Rendering engines are used to enable simulation of the perception stack.

 <h2>Overview</h2>

 Perception simulation is supported via the RenderEngine API. It provides the
 interface through which the geometries in the simulation can be rendered to
 three types of images: color, depth, and label. (For more details about the
 API, refer to the RenderEngine documentation.)

 Drake includes *two* implementations of that API:

   - RenderEngineVtk - A GPU-based rasterization renderer
   - RenderEngineOspray - A CPU-based ray-tracing renderer

 These implementations differ in many ways:

   - Support of the API (i.e., how complete is the implementation?)
   - Fidelity (how "realistic" are the images?)
   - Performance

 Picking the right renderer for your simulated sensors will be based on
 considering those differences and picking the trade-off that best suits your
 application.

 A summary is given in the table below (and the details are provided in the
 section below):

 | Render Engine      | API Support       | Fidelity                                                     | Performance |
 |:------------------:|:-----------------:|:------------------------------------------------------------:|:-----------:|
 | RenderEngineVtk    | Full              | No shadows, limited anti-aliasing                            | Fast        |
 | RenderEngineOspray | Color images only | Shadow-less, ray-traced shadows, or full global illumination | Slow        |

 <h2>API Support</h2>

 There's little to elaborate over that indicated by the table.

   - RenderEngineVtk: Full support, i.e. color, depth and label images.
   - RenderEngineOspray: *Currently* only supports color images, but the missing
     images are planned for the future.

 <h2>Fidelity</h2>

   - RenderEngineVtk
     - Uses a simple phong lighting model (although with fragment instead of
       vertex shading).
     - Currently no shadows. In the future, approximate real-time shadows will
       be introduced.
   - RenderEngineOspray
     - Supports shadow-less rendering (which should produce a near identical
       image to that of RenderEngineVtk).
     - Simple ray-traced shadows can be added (with respect to point lights).
     - Full path-tracing global illumination can be included to provide
       secondary illumination.

 The current API is simple and precludes sophisticated definitions of lights
 and materials. As the API matures, these additional features will be added and
 the fidelity differences between the two RenderEngine implementations will
 increase.

 <h2>Performance</h2>

 Generally, the GPU-based rasterization renderer is faster than the CPU-based
 ray-tracer. Even with shadows turned off, RenderEngineOspray is slower than
 RenderEngineVtk for the same image.

 The performance of RenderEngineOspray depends on the number of CPU resources
 available; it is multi-threaded and will make maximum use of those CPUs. More
 CPUs lead to shorter render times.

 The performance of RenderEngineVtk will depend on the quality of the GPU and
 its driver. A more powerful graphics card with an up-to-date driver will likely
 produce images more quickly.

 It should be clear that the relative performance between the two RenderEngine
 implementation depends on a particular system's configuration. As such, it is
 impossible to state what the two engine's relative performance is in absolute
 terms. Instead, Drake includes a simple @ref render_engine_benchmarks
 "benchmark" which can be run locally to assess the actual performance the end
 user will experience.

 */

/** @defgroup render_benchmarks Render Benchmarks
 @ingroup render_engines

 Benchmarks for assessing rendering performance and comparing implementations.

 */

}  // namespace render
}  // namespace geometry
}  // namespace drake

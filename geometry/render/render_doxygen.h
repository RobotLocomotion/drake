namespace drake {
namespace geometry {
namespace render {

/** @addtogroup render_engines

 <h2>Overview</h2>

 Perception simulation is supported via the RenderEngine API. It provides the
 interface through which the geometries in the simulation can be rendered to
 three types of images: color, depth, and label. (For more details about the
 API, refer to the RenderEngine documentation).

 Drake includes *two* implementations of that API:

   - RenderEngineVtk - A GPU-based rasterization renderer
   - RenderEngineOspray - A CPU-based ray-tracing renderer

 These implementations differ in many ways:

   - Support of the API (i.e., how complete is the implementation)
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
       image to that of RenderEngineVtk.
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
 terms. Instead, Drake includes a simple benchmark which can be run locally to
 assess the actual performance the end user will experience.

 <h3>Render engine benchmark</h3>

 <h4>Description</h4>

 The benchmark consists of a scene with a ground box, one or more spheres
 floating above the plane, and one or more cameras above the spheres, looking
 down at the spheres.

 If there are multiple spheres, they are positioned in a regular grid positioned
 at a uniform height above the ground plane. Increasing the number of spheres
 provides an approximate measure of how the renderer performs with increased
 scene complexity.

 If there are multiple cameras, they are all at the same position, looking in
 the same direction, with the same intrinsic properties. In other words, each
 should produce the same output image. This provides a measure of the
 scalability as a simulation includes an increasing number of cameras.

 The output image can be configured to an arbitrary size. For both RenderEngine
 implementations, larger images take more time.

 <h4>Running the benchmark</h4>

 The benchmark can be executed as:

 ```
 bazel run //geometry/benchmarking:render_benchmark
 ```

 The output will be something akin to:

 ```
Run on (12 X 4400 MHz CPU s)
CPU Caches:
  L1 Data 32K (x6)
  L1 Instruction 32K (x6)
  L2 Unified 256K (x6)
  L3 Unified 12288K (x1)
Load Average: 9.75, 4.65, 3.26
***WARNING*** CPU scaling is enabled, the benchmark real time measurements may be noisy and will incur extra overhead.
------------------------------------------------------------------------------------------------------
Benchmark                                                            Time             CPU   Iterations
------------------------------------------------------------------------------------------------------
RenderEngineBenchmark/VtkColor/1/1/640/480                        1.11 ms         1.08 ms          514
RenderEngineBenchmark/VtkColor/4/1/640/480                        1.05 ms         1.05 ms          684
RenderEngineBenchmark/VtkColor/8/1/640/480                        1.10 ms         1.09 ms          610
RenderEngineBenchmark/VtkColor/1/10/640/480                       10.5 ms         10.1 ms           67
RenderEngineBenchmark/VtkColor/1/1/320/240                       0.391 ms        0.390 ms         1567
RenderEngineBenchmark/VtkColor/1/1/1280/960                       3.13 ms         3.13 ms          223
RenderEngineBenchmark/VtkColor/1/1/2560/1920                      12.2 ms         12.2 ms           49
RenderEngineBenchmark/VtkDepth/1/1/640/480                        1.35 ms         1.35 ms          484
RenderEngineBenchmark/VtkDepth/1/10/640/480                       13.0 ms         13.0 ms           50
RenderEngineBenchmark/VtkLabel/1/1/640/480                        1.45 ms         1.45 ms          464
RenderEngineBenchmark/VtkLabel/1/10/640/480                       26.1 ms         25.3 ms           37
RenderEngineBenchmark/OsprayRayColor/1/1/640/480                  23.4 ms         22.7 ms           33
RenderEngineBenchmark/OsprayRayColor/4/1/640/480                  28.8 ms         27.4 ms           23
RenderEngineBenchmark/OsprayRayColor/8/1/640/480                  34.4 ms         32.9 ms           17
RenderEngineBenchmark/OsprayRayColor/1/10/640/480                  193 ms          174 ms            3
RenderEngineBenchmark/OsprayRayColor/1/1/320/240                  5.16 ms         5.09 ms          120
RenderEngineBenchmark/OsprayRayColor/1/1/1280/960                 67.9 ms         65.2 ms           11
RenderEngineBenchmark/OsprayRayColor/1/1/2560/1920                 283 ms          267 ms            2
RenderEngineBenchmark/OsprayRayColorShadowsOff/1/1/640/480        15.7 ms         15.7 ms           43
RenderEngineBenchmark/OsprayRayColorShadowsOff/4/1/640/480        20.0 ms         19.6 ms           30
RenderEngineBenchmark/OsprayRayColorShadowsOff/8/1/640/480        25.2 ms         25.2 ms           28
RenderEngineBenchmark/OsprayRayColorShadowsOff/1/10/640/480        162 ms          159 ms            4
RenderEngineBenchmark/OsprayPathColor/1/1/640/480                 36.2 ms         35.3 ms           20
RenderEngineBenchmark/OsprayPathColor/1/10/640/480                 351 ms          345 ms            2
 ```

 Additional configuration is possible via the following flags:
 - __save_image_path__: Enables saving the rendered images in the given
   location. Defaults to no saving.
 - __show_window__: Whether to display the rendered images. Defaults to false.
 - __samples_per_pixel__: The number of illumination samples per pixel when path
   tracing with RenderEngineOspray. Higher numbers introduce higher quality at
   increased cost. Defaults to 1.

 For example:
 ```
 bazel run //geometry/benchmarking:render_benchmark -- --save_image_path="/tmp" --show_window=true --samples_per_pixel=100
 ```

 <h4>Interpreting the benchmark</h4>

 Each line in the table represents a particular configuration of the benchmark
 parameters of the form:

 ```
 BenchmarkName/TestName/camera_count/sphere_count/image_width/image_height
 ```

   - __Benchmarkname__: `RenderEngineBenchmark` for all results in this
      executable.
   - __TestName__: One of
     - __VtkColor__: Renders the color image from RenderEngineVtk.
     - __VtkDepth__: Renders the depth image from RenderEngineVtk.
     - __VtkLabel__: Renders the label image from RenderEngineVtk.
     - __OsprayRayColor__: Renders the color image from RenderEngineOspray with
       ray-traced shadows.
     - __OsprayRayColorShadowsOff__: Renders the color image from
       RenderEngineOspray without any shadows -- most closely approximates the
       RenderEngineVtk color image.
     - __OsprayPathColor__: Renders the color image from RenderEngineOspray with
       path-traced global illumination (with only a single sample per pixel by
       default, unless configured using --samples_per_pixel).
   - __camera_count__: Simply the number of independent cameras being rendered.
     The cameras are all co-located (same position, same view direction) so
     they each render the same image.
   - __sphere_count__: The total number of spheres.
   - __image width__ and __image_height__: The dimensions of the output image
     in pixels.

 The `Time` and `CPU` columns are measures of the average time it took to create
 a single frame for *each camera*. The `Iterations` indicates how often the
 action was performed to compute the average value. For more information see the
 [google benchmark documentation](https://github.com/google/benchmark).

 Now we can analyze the example output and draw some example inferences (not a
 complete set of valid inferences):

   - Frame cost scales linearly with the number of cameras.
     - For RenderEngineVtk a 10X increase in number of cameras leads to a 10X
       increase in time for both color and depth, but a (roughly) 20X increase
       for label.
     - RenderEngineVtk also increased a factor of 10X when path-tracing the
       scene when we increased the number of cameras by a factor of 10X.
   - The number of objects in the scene has an apparently negligible impact on
     RenderEngineVtk, but a noticeable impact on RenderEngineOspray.

 */

}  // namespace render
}  // namespace geometry
}  // namespace drake

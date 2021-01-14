
namespace drake {
namespace geometry {
namespace render {

/** @defgroup render_engine_benchmarks Render Engine Benchmarks
 @ingroup render_benchmarks

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

 <h2>Running the benchmark</h2>

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
------------------------------------------------------------------------------------------------------  // NOLINT(*)
Benchmark                                                            Time             CPU   Iterations  // NOLINT(*)
------------------------------------------------------------------------------------------------------  // NOLINT(*)
RenderEngineBenchmark/VtkColor/1/1/640/480                        1.11 ms         1.08 ms          514  // NOLINT(*)
RenderEngineBenchmark/VtkColor/4/1/640/480                        1.05 ms         1.05 ms          684  // NOLINT(*)
RenderEngineBenchmark/VtkColor/8/1/640/480                        1.10 ms         1.09 ms          610  // NOLINT(*)
RenderEngineBenchmark/VtkColor/1/10/640/480                       10.5 ms         10.1 ms           67  // NOLINT(*)
RenderEngineBenchmark/VtkColor/1/1/320/240                       0.391 ms        0.390 ms         1567  // NOLINT(*)
RenderEngineBenchmark/VtkColor/1/1/1280/960                       3.13 ms         3.13 ms          223  // NOLINT(*)
RenderEngineBenchmark/VtkColor/1/1/2560/1920                      12.2 ms         12.2 ms           49  // NOLINT(*)
RenderEngineBenchmark/VtkDepth/1/1/640/480                        1.35 ms         1.35 ms          484  // NOLINT(*)
RenderEngineBenchmark/VtkDepth/1/10/640/480                       13.0 ms         13.0 ms           50  // NOLINT(*)
RenderEngineBenchmark/VtkLabel/1/1/640/480                        1.45 ms         1.45 ms          464  // NOLINT(*)
RenderEngineBenchmark/VtkLabel/1/10/640/480                       26.1 ms         25.3 ms           37  // NOLINT(*)
 ```

 Additional configuration is possible via the following flags:
 - __save_image_path__: Enables saving the rendered images in the given
   location. Defaults to no saving.
 - __show_window__: Whether to display the rendered images. Defaults to false.

 For example:
 ```
 bazel run //geometry/benchmarking:render_benchmark -- \
    --save_image_path="/tmp" --show_window=true --samples_per_pixel=100
 ```

 <h4>Interpreting the benchmark</h4>

 Each line in the table represents a particular configuration of the benchmark
 parameters of the form:

 ```
 RenderEngineBenchmark/TestName/camera_count/sphere_count/image_width/image_height
 ```

   - __TestName__: One of
     - __VtkColor__: Renders the color image from RenderEngineVtk.
     - __VtkDepth__: Renders the depth image from RenderEngineVtk.
     - __VtkLabel__: Renders the label image from RenderEngineVtk.
   - __camera_count__: Simply the number of independent cameras being rendered.
     The cameras are all co-located (same position, same view direction) so
     they each render the same image.
   - __sphere_count__: The total number of spheres.
   - __image width__ and __image_height__: The dimensions of the output image
     in pixels.

 The `Time` and `CPU` columns are measures of the average time it took to create
 a single frame for all the specified cameras. The `Iterations` indicates how
 often the action was performed to compute the average value. For more
 information see the [google benchmark
 documentation](https://github.com/google/benchmark).

 Now we can analyze the example output and draw some example inferences (not a
 complete set of valid inferences):

   - Frame cost scales linearly with the number of cameras.
     - For RenderEngineVtk a 10X increase in number of cameras leads to a 10X
       increase in time for both color and depth, but a (roughly) 20X increase
       for label.
     - RenderEngineVtk also increased a factor of 10X when path-tracing the
       scene when we increased the number of cameras by a factor of 10X.
 */
}  // namespace render
}  // namespace geometry
}  // namespace drake

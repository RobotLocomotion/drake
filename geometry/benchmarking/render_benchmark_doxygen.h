
namespace drake {
namespace geometry {
namespace render {

/** @defgroup render_engine_benchmarks Render Engine Benchmarks
 @ingroup render_benchmarks

 The benchmark consists of a scene with one or more spheres and one or more
 cameras above the spheres, looking down at the spheres.

 If there are multiple spheres, they are positioned in a regular grid placed
 at a uniform height above the ground plane. Increasing the number of spheres
 provides an approximate measure of how the renderer performs with increased
 scene complexity.

 If there are multiple cameras, they are all at the same position, looking in
 the same direction, with the same intrinsic properties. In other words, each
 should produce the same output image. This provides a measure of the
 scalability as a simulation includes an increasing number of cameras.

 The output image can be configured to an arbitrary size; larger images take
 more rendering time.

 <h2>Warm-starting RenderEngine Implementations</h2>

 Some RenderEngine implementations may defer some of the initialization work
 until the rendering API is actually invoked (e.g., RenderEngineVtk). In these
 cases, the first rendering will be misleadingly expensive. The benchmark
 "warm starts" each render engine under evaluation by performing two renderings
 outside of the timed loop. This allows the benchmark to better report the
 expected results in the engine's "steady state".

 When comparing these benchmark results with observed performance in
 applications, remember that the first and possibly second renderings may
 deceptively impact any analysis of *average* render performance.

 <h2>Benchmarks</h2>

 The benchmarks have been configured to examine the following characteristics of
 the included RenderEngine implementations:

   - Scalability w.r.t. scene complexity
     - For a fixed image size, we render a sequence of images with varying
       numbers of spheres.
   - Scalability w.r.t. number of cameras
     - For a fixed image size, render multiple cameras at the simplest and most
       complex scene complexities.
   - Scalability w.r.t. image size
     - With a single camera, we render multiple output images of varying sizes
       with both simple and complex scenes.

 We examine those same properties for all three image types: color, depth, and
 label.

 <h2>Running the benchmark</h2>

 The benchmark can be executed as:

 ```
 bazel run //geometry/benchmarking:render_benchmark
 ```

 The output will be something akin to the following (although not identical;
 lines have been removed for brevity):

 ```
 Run on (12 X 4400 MHz CPU s)
 CPU Caches:
   L1 Data 32K (x6)
   L1 Instruction 32K (x6)
   L2 Unified 256K (x6)
   L3 Unified 12288K (x1)
 Load Average: 9.75, 4.65, 3.26
 ------------------------------------------------------------------------------------  // NOLINT(*)
 Benchmark                                          Time             CPU   Iterations  // NOLINT(*)
 ------------------------------------------------------------------------------------  // NOLINT(*)
 RenderBenchmark/VtkColor/1/1/640/480           0.641 ms        0.641 ms         1026  // NOLINT(*)
 RenderBenchmark/VtkColor/12/1/640/480          0.843 ms        0.843 ms          882  // NOLINT(*)
 RenderBenchmark/VtkColor/120/1/640/480          2.58 ms         2.58 ms          280  // NOLINT(*)
 RenderBenchmark/VtkColor/240/1/640/480          5.30 ms         5.30 ms          100  // NOLINT(*)
 RenderBenchmark/VtkColor/480/1/640/480          9.10 ms         9.10 ms           79  // NOLINT(*)
 RenderBenchmark/VtkColor/1200/1/640/480         23.4 ms         23.4 ms           30  // NOLINT(*)
 RenderBenchmark/VtkColor/1/10/640/480           6.36 ms         6.36 ms          117  // NOLINT(*)
 RenderBenchmark/VtkColor/1200/10/640/480         179 ms          179 ms            4  // NOLINT(*)
 RenderBenchmark/VtkColor/1/1/320/240           0.267 ms        0.267 ms         2318  // NOLINT(*)
 RenderBenchmark/VtkColor/1/1/1280/960           2.39 ms         2.39 ms          317  // NOLINT(*)
 RenderBenchmark/VtkColor/1/1/2560/1920          11.5 ms         11.5 ms           65  // NOLINT(*)
 RenderBenchmark/VtkColor/1200/1/320/240         24.1 ms         24.0 ms           30  // NOLINT(*)
 RenderBenchmark/VtkColor/1200/1/1280/960        27.0 ms         26.9 ms           26  // NOLINT(*)
 RenderBenchmark/VtkColor/1200/1/2560/1920       39.8 ms         39.7 ms           19  // NOLINT(*)
 RenderBenchmark/GlColor/1/1/640/480            0.523 ms        0.480 ms         1382  // NOLINT(*)
 RenderBenchmark/GlColor/12/1/640/480           0.553 ms        0.511 ms         1236  // NOLINT(*)
 RenderBenchmark/GlColor/120/1/640/480          0.833 ms        0.790 ms          884  // NOLINT(*)
 RenderBenchmark/GlColor/240/1/640/480           1.20 ms         1.15 ms          591  // NOLINT(*)
 RenderBenchmark/GlColor/480/1/640/480           1.87 ms         1.83 ms          384  // NOLINT(*)
 RenderBenchmark/GlColor/1200/1/640/480          3.76 ms         3.72 ms          188  // NOLINT(*)
 RenderBenchmark/GlColor/1/10/640/480            4.98 ms         4.56 ms          137  // NOLINT(*)
 RenderBenchmark/GlColor/1200/10/640/480         37.1 ms         36.6 ms           19  // NOLINT(*)
 RenderBenchmark/GlColor/1/1/320/240            0.151 ms        0.147 ms         4830  // NOLINT(*)
 RenderBenchmark/GlColor/1/1/1280/960            1.93 ms         1.88 ms          335  // NOLINT(*)
 RenderBenchmark/GlColor/1/1/2560/1920           9.14 ms         9.08 ms           72  // NOLINT(*)
 RenderBenchmark/GlColor/1200/1/320/240          3.46 ms         3.42 ms          199  // NOLINT(*)
 RenderBenchmark/GlColor/1200/1/1280/960         5.29 ms         5.25 ms          127  // NOLINT(*)
 RenderBenchmark/GlColor/1200/1/2560/1920        12.5 ms         12.5 ms           57  // NOLINT(*)
 RenderBenchmark/VtkDepth/1/1/640/480            1.89 ms         1.89 ms          391  // NOLINT(*)
 RenderBenchmark/GlDepth/1/1/640/480            0.486 ms        0.486 ms         1578  // NOLINT(*)
 RenderBenchmark/VtkLabel/1/1/640/480            1.35 ms         1.35 ms          513  // NOLINT(*)
 RenderBenchmark/GlLabel/1/1/640/480             1.34 ms         1.30 ms          521  // NOLINT(*)
 ```

 Additional configuration is possible via the following flags:
 - __save_image_path__: Enables saving the rendered images in the given
   location. Defaults to no saving.
 - __show_window__: Whether to display the rendered images. Defaults to false.

 For example:
 ```
 bazel run //geometry/benchmarking:render_benchmark -- \
    --save_image_path="/tmp" --show_window=true
 ```

 <h4>Interpreting the benchmark</h4>

 Each line in the table represents a particular configuration of the benchmark
 parameters of the form:

 ```
 RenderBenchmark/TestName/sphere_count/camera_count/image_width/image_height
 ```

   - __TestName__: One of
     - __VtkColor__: Renders the color image from RenderEngineVtk.
     - __VtkDepth__: Renders the depth image from RenderEngineVtk.
     - __VtkLabel__: Renders the label image from RenderEngineVtk.
     - __GlColor__: Renders the color image from RenderEngineGl.
     - __GlDepth__: Renders the depth image from RenderEngineGl.
     - __GlLabel__: Renders the label image from RenderEngineGl.
   - __sphere_count__: The total number of spheres.
   - __camera_count__: Simply the number of independent cameras being rendered.
     The cameras are all co-located (same position, same view direction) so
     they each render the same image.
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
   - RenderEngineGl scales better with respect to scene complexity than
     RenderEngineVtk.
 */
}  // namespace render
}  // namespace geometry
}  // namespace drake

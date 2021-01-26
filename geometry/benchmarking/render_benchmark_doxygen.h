
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
 RenderBenchmark/VtkColor/1/1/640/480           0.704 ms        0.694 ms          988  // NOLINT(*)
 RenderBenchmark/VtkColor/12/1/640/480           1.08 ms         1.06 ms          731  // NOLINT(*)
 RenderBenchmark/VtkColor/120/1/640/480          4.74 ms         4.59 ms          115  // NOLINT(*)
 RenderBenchmark/VtkColor/1200/1/640/480          857 ms          845 ms            1  // NOLINT(*)
 RenderBenchmark/VtkColor/1/10/640/480           6.99 ms         6.84 ms           97  // NOLINT(*)
 RenderBenchmark/VtkColor/1200/10/640/480         951 ms          932 ms            1  // NOLINT(*)
 RenderBenchmark/VtkColor/1/1/320/240           0.268 ms        0.262 ms         2304  // NOLINT(*)
 RenderBenchmark/VtkColor/1/1/1280/960           2.79 ms         2.70 ms          213  // NOLINT(*)
 RenderBenchmark/VtkColor/1/1/2560/1920          12.2 ms         11.9 ms           59  // NOLINT(*)
 RenderBenchmark/VtkColor/1200/1/320/240          681 ms          663 ms            1  // NOLINT(*)
 RenderBenchmark/VtkColor/1200/1/1280/960         681 ms          668 ms            1  // NOLINT(*)
 RenderBenchmark/VtkColor/1200/1/2560/1920        715 ms          698 ms            1  // NOLINT(*)
 RenderBenchmark/GlColor/1/1/640/480            0.576 ms        0.489 ms         1309  // NOLINT(*)
 RenderBenchmark/GlColor/12/1/640/480           0.624 ms        0.539 ms         1173  // NOLINT(*)
 RenderBenchmark/GlColor/120/1/640/480          0.922 ms        0.814 ms          847  // NOLINT(*)
 RenderBenchmark/GlColor/1200/1/640/480          4.13 ms         4.08 ms          161  // NOLINT(*)
 RenderBenchmark/GlColor/1/10/640/480            5.66 ms         4.80 ms          141  // NOLINT(*)
 RenderBenchmark/GlColor/1200/10/640/480         42.1 ms         41.6 ms           15  // NOLINT(*)
 RenderBenchmark/GlColor/1/1/320/240            0.170 ms        0.151 ms         4617  // NOLINT(*)
 RenderBenchmark/GlColor/1/1/1280/960            2.20 ms         2.16 ms          343  // NOLINT(*)
 RenderBenchmark/GlColor/1/1/2560/1920           10.5 ms         9.62 ms           74  // NOLINT(*)
 RenderBenchmark/GlColor/1200/1/320/240          3.74 ms         3.41 ms          205  // NOLINT(*)
 RenderBenchmark/GlColor/1200/1/1280/960         5.93 ms         5.89 ms          108  // NOLINT(*)
 RenderBenchmark/GlColor/1200/1/2560/1920        14.6 ms         14.5 ms           36  // NOLINT(*)
 RenderBenchmark/VtkDepth/1/1/640/480            1.69 ms         1.64 ms          424  // NOLINT(*)
 RenderBenchmark/GlDepth/1/1/640/480            0.541 ms        0.541 ms         1220  // NOLINT(*)
 RenderBenchmark/VtkLabel/1/1/640/480            1.65 ms         1.60 ms          339  // NOLINT(*)
 RenderBenchmark/GlLabel/1/1/640/480             1.77 ms         1.72 ms          334  // NOLINT(*)
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

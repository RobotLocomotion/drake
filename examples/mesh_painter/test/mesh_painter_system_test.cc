#include "drake/examples/mesh_painter/mesh_painter_system.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "common/find_resource.h"

namespace drake {
namespace examples {
namespace mesh_painter {
namespace {

using geometry::Box;
using geometry::Capsule;
using geometry::Convex;
using geometry::Cylinder;
using geometry::Ellipsoid;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::HalfSpace;
using geometry::Mesh;
using geometry::ProximityProperties;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::ShapeName;
using geometry::SourceId;
using geometry::Sphere;
using math::RigidTransformd;
using systems::DiagramBuilder;
using systems::sensors::ImageRgba8U;
using Eigen::Vector3d;
using std::make_pair;
using std::make_unique;
using std::optional;
using std::pair;
using std::string;

constexpr double kPeriodSec = 0.05;

class MeshPainterSystemTest : public ::testing::Test {
 protected:
  void SetUp() override {
    source_id_ = scene_graph_.RegisterSource("this_test");
    const std::string file_name =
        FindAnzuResourceOrThrow("intuitive/sim/test/meshes/textured_quad.obj");
    convex_ = Convex{file_name, 100.0};
    // NOTE: This has to be a legitimate file, otherwise the MeshPainterSystem
    // constructor will fail in creating a canvas representation.
    mesh_ = Mesh{file_name, 100.0};
  }

  /* Registers the given geometry. If a scene graph is given, it registers it
   to *that* scene graph. Otherwise, registers it to the member scene graph.
   The geometry is registered to a newly registered frame and both frame and
   geoemtry ids are returned.  */
  template <typename ShapeType>
  pair<FrameId, GeometryId> AddShapeToSceneGraphBothIds(
      const ShapeType& shape, SceneGraph<double>* sg = nullptr) {
    if (sg == nullptr) sg = &scene_graph_;
    const FrameId frame_id =
        sg->RegisterFrame(source_id_, GeometryFrame(NextName()));
    const GeometryId geometry_id = sg->RegisterGeometry(
        source_id_, frame_id,
        make_unique<GeometryInstance>(
            RigidTransformd{}, make_unique<ShapeType>(shape), NextName()));
    return make_pair(frame_id, geometry_id);
  }

  /* Variant of AddShapeToSceneGraphBothIds() but only returns the GeometryId.
   */
  template <typename ShapeType>
  GeometryId AddShapeToSceneGraph(const ShapeType& shape,
                                  SceneGraph<double>* sg = nullptr) {
    auto pair = AddShapeToSceneGraphBothIds(shape, sg);
    return pair.second;
  }

  /* Generates a sequence of unique names. */
  static string NextName() {
    static int frame_count = 0;
    return fmt::format("Name{}", frame_count++);
  }

  /* Tests the validity of the given shape in a painter/canvas role. The caller
   *must* provide an id for one and only one of the painter and canvas roles.
   The shape will be used to define the missing id for the other role.
   If an error message is provided, an exception is expected with an error
   message that will match the message. Otherwise, no throw is expected.  */
  template <typename ShapeType>
  void TestShapeType(optional<GeometryId> painter_id,
                     optional<GeometryId> canvas_id, const ShapeType& shape,
                     const string& error_message) {
    ASSERT_NE(painter_id.has_value(), canvas_id.has_value());
    const GeometryId g_id = AddShapeToSceneGraph(shape);
    if (!painter_id.has_value()) {
      painter_id = g_id;
    } else {
      canvas_id = g_id;
    }
    if (!error_message.empty()) {
      DRAKE_EXPECT_THROWS_MESSAGE(
          MeshPainterSystem(*canvas_id, *painter_id, 4, 4, scene_graph_,
                            kPeriodSec),
          error_message);
    } else {
      EXPECT_NO_THROW(MeshPainterSystem(*canvas_id, *painter_id, 4, 4,
                                        scene_graph_, kPeriodSec));
    }
  }

  // Examines all pixels in the image; if any pixel is not (255, 255, 255, 255),
  // returns false.
  bool ImageIsAllWhite(const ImageRgba8U& image) {
    for (int c = 0; c < image.width(); ++c) {
      for (int r = 0; r < image.height(); ++r) {
        const auto* pixel = image.at(c, r);
        if (pixel[0] != 255 || pixel[1] != 255 || pixel[2] != 255 ||
            pixel[3] != 255) {
          return false;
        }
      }
    }
    return true;
  }

  SceneGraph<double> scene_graph_;
  // A collection of pre-instantiated shapes; each should be valid but the
  // actual parameter values don't matter.
  Box box_{1, 1, 1};
  Capsule capsule_{1, 1};
  Convex convex_{"", 1};
  Cylinder cylinder_{1, 1};
  Ellipsoid ellipsoid{1, 2, 3};
  HalfSpace half_space_;
  Mesh mesh_{"", 1};
  Sphere sphere_{1};

  SourceId source_id_{};
};

// Confirms that only Mesh is supported and all other Shapes give a meaningful
// error message.
TEST_F(MeshPainterSystemTest, SupportedCanvasShapes) {
#define ScopedCanvasTest(shape, should_throw)                              \
  {                                                                        \
    SCOPED_TRACE(ShapeName(shape).name());                                 \
    string error_message = "";                                             \
    if (should_throw) {                                                    \
      error_message = fmt::format(                                         \
          "MeshPainterSystem .+ Mesh shape types as canvas; {} specified", \
          ShapeName(shape).name());                                        \
    }                                                                      \
    TestShapeType(painter_id, {}, shape, error_message);                   \
  }

  // Create a painter known to be valid.
  const GeometryId painter_id = AddShapeToSceneGraph(cylinder_);

  // Only the mesh is a valid canvas.
  ScopedCanvasTest(mesh_, false /* should throw */);

  // All other shapes are invalid canvases.
  ScopedCanvasTest(box_, true /* should throw */);
  ScopedCanvasTest(capsule_, true /* should throw */);
  ScopedCanvasTest(convex_, true /* should throw */);
  ScopedCanvasTest(cylinder_, true /* should throw */);
  ScopedCanvasTest(ellipsoid, true /* should throw */);
  ScopedCanvasTest(half_space_, true /* should throw */);
  ScopedCanvasTest(sphere_, true /* should throw */);

#undef ScopedCanvasTest
}

// Confirms that supported shapes work and unsupported shapes throw a meaningful
// error message.
TEST_F(MeshPainterSystemTest, SupportedPainterShapes) {
#define ScopedPainterTest(shape, should_throw)                             \
  {                                                                        \
    SCOPED_TRACE(ShapeName(shape).name());                                 \
    string error_message = "";                                             \
    if (should_throw) {                                                    \
      error_message = fmt::format(                                         \
          "MeshPainterSystem only supports Box, Cylinder, Ellipsoid, and " \
          "Sphere as painter objects; {} specified",                       \
          ShapeName(shape).name());                                        \
    }                                                                      \
    TestShapeType({}, canvas_id, shape, error_message);                    \
  }

  // Create a canvas known to be valid.
  const GeometryId canvas_id = AddShapeToSceneGraph(mesh_);

  // Supported painter shapes.
  ScopedPainterTest(box_, false /* should throw */);
  ScopedPainterTest(cylinder_, false /* should throw */);
  ScopedPainterTest(ellipsoid, false /* should throw */);
  ScopedPainterTest(sphere_, false /* should throw */);

  // Unsupported painter shapes.
  ScopedPainterTest(mesh_, true /* should throw */);
  ScopedPainterTest(capsule_, true /* should throw */);
  ScopedPainterTest(convex_, true /* should throw */);
  ScopedPainterTest(half_space_, true /* should throw */);

#undef ScopedPainterTest
}

// Confirms that OBJs that don't have sufficient/correct data to serve as a
// canvas geometry are reported with meaningful exception messages.
TEST_F(MeshPainterSystemTest, BadObjs) {
  const GeometryId painter_id = AddShapeToSceneGraph(Sphere{1.0});

  {
    // Not really an obj.
    const GeometryId g_id = AddShapeToSceneGraph(Mesh{
        FindAnzuResourceOrThrow("intuitive/sim/test/meshes/not_an_obj.obj"),
        1.0});
    DRAKE_EXPECT_THROWS_MESSAGE(
        MeshPainterSystem(g_id, painter_id, 1, 1, scene_graph_, kPeriodSec),
        "Parsing file as OBJ file produced no geometry: .+");
  }
  {
    // No faces in OBJ.
    const GeometryId g_id = AddShapeToSceneGraph(
        Mesh{FindAnzuResourceOrThrow("intuitive/sim/test/meshes/no_faces.obj"),
             1.0});
    DRAKE_EXPECT_THROWS_MESSAGE(
        MeshPainterSystem(g_id, painter_id, 1, 1, scene_graph_, kPeriodSec),
        "Parsing file as OBJ file produced no geometry: .+");
  }
  {
    // No uvs on faces in OBJ.
    const GeometryId g_id = AddShapeToSceneGraph(Mesh{
        FindAnzuResourceOrThrow("intuitive/sim/test/meshes/no_uvs.obj"), 1.0});
    DRAKE_EXPECT_THROWS_MESSAGE(
        MeshPainterSystem(g_id, painter_id, 1, 1, scene_graph_, kPeriodSec),
        "OBJ cannot be used as a paint canvas; it has no texture coordinates: "
        ".+");
  }
  {
    // Not all faces have uvs in OBJ.
    const GeometryId g_id = AddShapeToSceneGraph(Mesh{
        FindAnzuResourceOrThrow("intuitive/sim/test/meshes/partial_uvs.obj"),
        1.0});
    DRAKE_EXPECT_THROWS_MESSAGE(
        MeshPainterSystem(g_id, painter_id, 1, 1, scene_graph_, kPeriodSec),
        "OBJ cannot be used as a paint canvas; at least one face is missing "
        "texture coordinates: .+");
  }
  {
    // OBJ has texture coordinates that span periodic image boundaries.
    const GeometryId g_id = AddShapeToSceneGraph(Mesh{
        FindAnzuResourceOrThrow("intuitive/sim/test/meshes/spanning_uvs.obj"),
        1.0});
    DRAKE_EXPECT_THROWS_MESSAGE(
        MeshPainterSystem(g_id, painter_id, 1, 1, scene_graph_, kPeriodSec),
        "OBJ cannot be used as a paint canvas; at least one triangle spans "
        "image boundaries: .+");
  }
  {
    // OBJ has negative texture coordinates.
    const GeometryId g_id = AddShapeToSceneGraph(Mesh{
        FindAnzuResourceOrThrow("intuitive/sim/test/meshes/negative_uvs.obj"),
        1.0});
    DRAKE_EXPECT_THROWS_MESSAGE(
        MeshPainterSystem(g_id, painter_id, 1, 1, scene_graph_, kPeriodSec),
        "OBJ cannot be used as a paint canvas; at least one texture coordinate "
        "has negative measures: .+");
  }
}

// Confirms that the image initializes to white with size input.
TEST_F(MeshPainterSystemTest, InitialImageIsWhiteWithSizeInput) {
  DiagramBuilder<double> builder;
  auto& sg = *builder.AddSystem<SceneGraph<double>>();
  source_id_ = sg.RegisterSource("ImageIsWhite");
  const auto [canvas_f_id, canvas_g_id] =
      AddShapeToSceneGraphBothIds(mesh_, &sg);
  const auto [paint_f_id, paint_g_id] =
      AddShapeToSceneGraphBothIds(cylinder_, &sg);
  auto& paint_system = *builder.AddSystem<MeshPainterSystem>(
      canvas_g_id, paint_g_id, 2, 2, sg, kPeriodSec);
  builder.Connect(sg.get_query_output_port(),
                  paint_system.geometry_query_input_port());
  builder.ExportInput(sg.get_source_pose_port(source_id_));
  builder.ExportOutput(paint_system.texture_output_port());
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  // Make sure the painter cylinder is far away from the canvas; no collision.
  const RigidTransformd X_WP{Vector3d{0, 0, 50}};
  const FramePoseVector<double> poses{{canvas_f_id, RigidTransformd{}},
                                      {paint_f_id, X_WP}};
  diagram->get_input_port(0).FixValue(context.get(), poses);
  const auto& image = diagram->get_output_port(0).Eval<ImageRgba8U>(*context);
  EXPECT_TRUE(ImageIsAllWhite(image));
}

// Confirms that the image initializes properly with image input.
TEST_F(MeshPainterSystemTest, InitialImageWithImageInput) {
  DiagramBuilder<double> builder;
  auto& sg = *builder.AddSystem<SceneGraph<double>>();
  source_id_ = sg.RegisterSource("TestSource");
  const auto [canvas_f_id, canvas_g_id] =
      AddShapeToSceneGraphBothIds(mesh_, &sg);
  const auto [paint_f_id, paint_g_id] =
      AddShapeToSceneGraphBothIds(cylinder_, &sg);

  constexpr int kSquareImageMeasure = 2;
  // Default initial value for all the bytes in the image.
  constexpr int kDefaultBytesValue = 255;
  // A testing byte value that is different with the default value.
  constexpr int kTestBytesValue = 0;
  ImageRgba8U initial_image(kSquareImageMeasure, kSquareImageMeasure,
                            kDefaultBytesValue);
  // Set the first pixel to a different value for testing.
  initial_image.at(0, 0)[0] = kTestBytesValue;
  auto& paint_system = *builder.AddSystem<MeshPainterSystem>(
      canvas_g_id, paint_g_id, initial_image, sg, kPeriodSec);
  builder.Connect(sg.get_query_output_port(),
                  paint_system.geometry_query_input_port());
  builder.ExportInput(sg.get_source_pose_port(source_id_));
  builder.ExportOutput(paint_system.texture_output_port());
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  // Make sure the painter cylinder is far away from the canvas; no collision.
  const RigidTransformd X_WP{Vector3d{0, 0, 50}};
  const FramePoseVector<double> poses{{canvas_f_id, RigidTransformd{}},
                                      {paint_f_id, X_WP}};
  diagram->get_input_port(0).FixValue(context.get(), poses);
  const auto& image = diagram->get_output_port(0).Eval<ImageRgba8U>(*context);
  EXPECT_FALSE(ImageIsAllWhite(image));
  // Confirm the output image has the correct size.
  EXPECT_EQ(image.width(), kSquareImageMeasure);
  EXPECT_EQ(image.width(), kSquareImageMeasure);

  // Check the initial output image matches the input image byte-by-byte.
  EXPECT_EQ(0,
            std::memcmp(image.at(0, 0), initial_image.at(0, 0), image.size()));
}

using systems::Context;
using systems::Diagram;
using systems::Simulator;

// Step forward (by a fraction of kPeriodSec) to tickle the discrete update for
// the mesh painter. This only takes state changes, and ignores changes from
// time and parameters.
void StepForward(const Diagram<double>& diagram, Context<double>* context) {
  // Clone the context so we can use input port fixtures.
  Simulator<double> simulator(diagram, context->Clone());
  // Step forward for whatever amount will tickle the update.
  simulator.AdvanceTo(kPeriodSec / 10.);
  // Only remember the state.
  context->get_mutable_state().SetFrom(simulator.get_context().get_state());
}

// Confirms that contact isn't enough to affect the image; the port must be
// evaluated while there is contact.
TEST_F(MeshPainterSystemTest, PaintOnUpdate) {
  // Configures a scenario with for painting on a texture.
  DiagramBuilder<double> builder;
  auto& sg = *builder.AddSystem<SceneGraph<double>>();
  source_id_ = sg.RegisterSource("ImageIsWhite");
  const auto [canvas_f_id, canvas_g_id] =
      AddShapeToSceneGraphBothIds(mesh_, &sg);
  const auto [paint_f_id, paint_g_id] =
      AddShapeToSceneGraphBothIds(cylinder_, &sg);

  // In order to use HasCollisions to confirm colliding state, the canvas and
  // painter need proximity roles. However, the canvas is a mesh which has no
  // proximity role support. So, we'll add a half space (on which the mesh lies)
  // to serve as proxy.
  const GeometryId canvas_collide_id = sg.RegisterGeometry(
      source_id_, canvas_f_id,
      make_unique<GeometryInstance>(RigidTransformd{}, make_unique<HalfSpace>(),
                                    "canvas_collision"));
  sg.AssignRole(source_id_, canvas_collide_id, ProximityProperties{});
  sg.AssignRole(source_id_, paint_g_id, ProximityProperties{});

  auto& paint_system = *builder.AddSystem<MeshPainterSystem>(
      canvas_g_id, paint_g_id, 2, 2, sg, kPeriodSec);
  builder.Connect(sg.get_query_output_port(),
                  paint_system.geometry_query_input_port());
  builder.ExportInput(sg.get_source_pose_port(source_id_));
  builder.ExportOutput(paint_system.texture_output_port());
  builder.ExportOutput(sg.get_query_output_port());
  auto diagram = builder.Build();

  const auto& image_port = diagram->get_output_port(0);
  const auto& pose_port = diagram->get_input_port(0);
  const auto& query_object_port = diagram->get_output_port(1);

  auto context = diagram->CreateDefaultContext();

  const RigidTransformd X_WP_separated{Vector3d{0, 0, 50}};
  const RigidTransformd X_WP_colliding{Vector3d{0, 0, 0}};
  FramePoseVector<double> poses{{canvas_f_id, RigidTransformd{}},
                                {paint_f_id, X_WP_separated}};

  // Confirm we start all white.
  pose_port.FixValue(context.get(), poses);
  ASSERT_FALSE(
      query_object_port.Eval<QueryObject<double>>(*context).HasCollisions());
  ASSERT_TRUE(ImageIsAllWhite(image_port.Eval<ImageRgba8U>(*context)));

  // We know the image starts all white (see test InitialImageIsWhite). We
  // move the painter into collision, confirm collision, then move them out of
  // collision. The image should *still* be white. If we move *back* into the
  // colliding configuration and then *evaluate* the image output port, the
  // image will no longer be all white.

  EXPECT_NE(sg.model_inspector().GetProximityProperties(canvas_collide_id),
            nullptr);
  EXPECT_NE(sg.model_inspector().GetProximityProperties(paint_g_id), nullptr);

  // Move painter into and out of collision -- image is unchanged.
  poses.set_value(paint_f_id, X_WP_colliding);
  pose_port.FixValue(context.get(), poses);
  ASSERT_TRUE(
      query_object_port.Eval<QueryObject<double>>(*context).HasCollisions());
  poses.set_value(paint_f_id, X_WP_separated);
  pose_port.FixValue(context.get(), poses);
  ASSERT_FALSE(
      query_object_port.Eval<QueryObject<double>>(*context).HasCollisions());
  ASSERT_TRUE(ImageIsAllWhite(image_port.Eval<ImageRgba8U>(*context)));

  // Move back into collision, and evaluate the image port.
  poses.set_value(paint_f_id, X_WP_colliding);
  pose_port.FixValue(context.get(), poses);
  StepForward(*diagram, context.get());
  ASSERT_TRUE(
      query_object_port.Eval<QueryObject<double>>(*context).HasCollisions());
  ASSERT_FALSE(ImageIsAllWhite(image_port.Eval<ImageRgba8U>(*context)));
}

// Confirms that the contact patch rasterizes properly.
TEST_F(MeshPainterSystemTest, PatchRasterization) {
  /* The textured_quad.obj mesh spans a box from [-0,5, -0.5] - [0.5, 0.5] on
   the x-y plane. It has been scaled by a factor of 100. If we assign a texture
   of 100 x 100 pixels to it, each pixel will be 1x1 meters. The *centers* of
   each pixel are at the half-meter marks, e.g., [-49.5, -49.5], ...,
   [49.5, 49.5]. Therefore, by carefully positioning a box painter, we should
   be able to predict exactly which pixel centers are inside the contact patch
   predict what should be rasterized. */
  DiagramBuilder<double> builder;
  auto& sg = *builder.AddSystem<SceneGraph<double>>();
  source_id_ = sg.RegisterSource("PatchRasterization");

  ASSERT_EQ(mesh_.scale(), 100.0);
  // Note: As of 5/9/2020, clang does not allow capturing variables created in
  // structured bindings to be captured in lambdas.
  GeometryId canvas_g_id;
  FrameId canvas_f_id;
  std::tie(canvas_f_id, canvas_g_id) = AddShapeToSceneGraphBothIds(mesh_, &sg);
  // Note: The box's cross section is smaller than a pixel; we can have
  // collision without actually enclosding a pixel center.
  GeometryId paint_g_id;
  FrameId paint_f_id;
  std::tie(paint_f_id, paint_g_id) =
      AddShapeToSceneGraphBothIds(Box{0.4, 0.4, 2.0}, &sg);
  auto& paint_system = *builder.AddSystem<MeshPainterSystem>(
      canvas_g_id, paint_g_id, 100, 100, sg, kPeriodSec);
  builder.Connect(sg.get_query_output_port(),
                  paint_system.geometry_query_input_port());
  builder.ExportInput(sg.get_source_pose_port(source_id_));
  builder.ExportOutput(paint_system.texture_output_port());
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  auto set_painter_pose = [canvas_f_id, paint_f_id, &diagram,
                           &context](const Vector3d& p_WP) {
    const FramePoseVector<double> poses{{canvas_f_id, RigidTransformd{}},
                                        {paint_f_id, RigidTransformd{p_WP}}};
    diagram->get_input_port(0).FixValue(context.get(), poses);
  };

  auto get_image = [&diagram, &context]() {
    StepForward(*diagram, context.get());
    return diagram->get_output_port(0).Eval<ImageRgba8U>(*context);
  };

  // Initial pose of painter is far away from the plane.
  set_painter_pose({0, 0, 10});
  EXPECT_TRUE(ImageIsAllWhite(get_image()));

  // Putting the box at the origin will cause the contact patch to lie between
  // pixel centers -- the image should still all be white.
  set_painter_pose({0, 0, 0});
  EXPECT_TRUE(ImageIsAllWhite(get_image()));

  // The quad's corners at (-50, -50), (-50, 50), (50, 50), and (50, -50) map
  // to texture coordaintes (0, 0), (0, 1), (1, 1), and (1, 0), respectively.
  // Place the box on the center of each of the corner pixels (0.5 unit in from
  // the image boundary) and confirm that the corresponding pixel gets colored.
  // We only test the red-component of the the pixel as a positive indicator.
  EXPECT_EQ(*get_image().at(0, 0), 255);
  set_painter_pose({-49.5, -49.5, 0});
  EXPECT_EQ(*get_image().at(0, 0), 0);

  EXPECT_EQ(*get_image().at(0, 99), 255);
  set_painter_pose({-49.5, 49.5, 0});
  EXPECT_EQ(*get_image().at(0, 99), 0);

  EXPECT_EQ(*get_image().at(99, 99), 255);
  set_painter_pose({49.5, 49.5, 0});
  EXPECT_EQ(*get_image().at(99, 99), 0);

  EXPECT_EQ(*get_image().at(99, 0), 255);
  set_painter_pose({49.5, -49.5, 0});
  EXPECT_EQ(*get_image().at(99, 0), 0);
}

}  // namespace
}  // namespace mesh_painter
}  // namespace examples
}  // namespace drake

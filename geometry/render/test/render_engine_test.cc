#include "drake/geometry/render/render_engine.h"

#include <memory>
#include <optional>
#include <set>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/test_utilities/dummy_render_engine.h"

namespace drake {
namespace geometry {
namespace render {

class RenderEngineTester {
 public:
  explicit RenderEngineTester(const RenderEngine* engine)
      : engine_(DRAKE_DEREF(engine)) {}

  int num_geometries() const {
    return static_cast<int>(engine_.update_ids_.size() +
                            engine_.anchored_ids_.size() +
                            engine_.deformable_mesh_dofs_.size());
  }

  bool has_id(GeometryId id) const {
    return engine_.update_ids_.contains(id) ||
           engine_.anchored_ids_.contains(id) ||
           engine_.deformable_mesh_dofs_.contains(id);
  }

 private:
  const RenderEngine& engine_;
};

namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::internal::DummyRenderEngine;
using math::RigidTransformd;
using std::set;
using std::unordered_map;
using systems::sensors::CameraInfo;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

// This is the absolutely minimum code to make an instantiable RenderEngine.
// It is *not* an example appropriate implementation of RenderEngine, it is
// merely a class of convenience for these tests so that individual aspects
// can be implemented in a targeted manner without worrying about this baseline
// functionality.
// This mainline implementation:
//    - UpdateViewpoint and DoUpdateVisualPose are no-ops.
//    - DoRegisterVisual and DoRemoveGeometry do nothing and report such.
//    - DoClone simply returns nullptr.
//    - Implements no rendering interfaces; invocations of
//      MinimumEngine::Render*Image (in any variant) will throw.
class MinimumEngine : public RenderEngine {
 public:
  void UpdateViewpoint(const math::RigidTransformd&) override {}
  bool DoRegisterVisual(GeometryId, const Shape&, const PerceptionProperties&,
                        const RigidTransformd&) override {
    return false;
  }
  void DoUpdateVisualPose(GeometryId, const math::RigidTransformd&) override {}
  bool DoRemoveGeometry(GeometryId) override { return false; }
  std::unique_ptr<RenderEngine> DoClone() const override { return nullptr; }
};

GTEST_TEST(RenderEngine, DeformableGeometryRegistrationAndUpdate) {
  DummyRenderEngine engine;

  geometry::internal::RenderMesh mesh;
  mesh.positions.resize(1, 3);
  mesh.normals.resize(1, 3);
  mesh.uvs.resize(1, 2);
  const Vector3d initial_q = Vector3d(1, 2, 3);
  const Vector3d initial_normal = Vector3d(-1, 0, 0);
  mesh.positions.row(0) = initial_q;
  mesh.normals.row(0) = initial_normal;
  mesh.uvs.row(0) = Vector2d(1, 0).transpose();

  PerceptionProperties properties = engine.accepting_properties();
  const GeometryId id0 = GeometryId::get_new_id();
  const GeometryId id1 = GeometryId::get_new_id();
  const GeometryId id2 = GeometryId::get_new_id();
  engine.RegisterDeformableVisual(id0, {mesh}, properties);
  // Throw if id is already registered.
  EXPECT_THROW(engine.RegisterDeformableVisual(id0, {mesh}, properties),
               std::exception);
  // Throw if no mesh is specified.
  EXPECT_THROW(engine.RegisterDeformableVisual(id1, {}, properties),
               std::exception);
  // Register a geometry with multiple meshes.
  engine.RegisterDeformableVisual(id1, {mesh, mesh}, properties);

  // Register another geometry with a property that prevents the registration.
  const PerceptionProperties skip_properties = engine.rejecting_properties();
  engine.RegisterDeformableVisual(id2, {mesh}, skip_properties);

  EXPECT_EQ(engine.num_registered(), 2);
  EXPECT_TRUE(engine.is_registered(id0));
  EXPECT_TRUE(engine.is_registered(id1));
  EXPECT_TRUE(engine.has_geometry(id0));
  EXPECT_TRUE(engine.has_geometry(id1));
  // Verify that geometry with arbitrary id is not registered.
  EXPECT_FALSE(engine.is_registered(GeometryId::get_new_id()));
  EXPECT_FALSE(engine.has_geometry(GeometryId::get_new_id()));
  // Verify that geometry with id2 is not registered.
  EXPECT_FALSE(engine.is_registered(id2));
  EXPECT_FALSE(engine.has_geometry(id2));

  {
    const std::vector<VectorXd>& q0 = engine.world_configurations(id0);
    ASSERT_EQ(q0.size(), 1);
    EXPECT_EQ(q0[0], initial_q);
    const std::vector<VectorXd>& q1 = engine.world_configurations(id1);
    ASSERT_EQ(q1.size(), 2);
    EXPECT_EQ(q1[0], initial_q);
    EXPECT_EQ(q1[1], initial_q);
  }

  {
    const std::vector<VectorXd>& n0 = engine.world_normals(id0);
    ASSERT_EQ(n0.size(), 1);
    EXPECT_EQ(n0[0], initial_normal);
    const std::vector<VectorXd>& n1 = engine.world_normals(id1);
    ASSERT_EQ(n1.size(), 2);
    EXPECT_EQ(n1[0], initial_normal);
    EXPECT_EQ(n1[1], initial_normal);
  }

  // Update id0 but not id1.
  VectorXd new_q(3);
  VectorXd new_normal(3);
  new_q << 4, 5, 6;
  new_normal << 7, 8, 9;
  new_normal.normalize();
  engine.UpdateDeformableConfigurations(id0, {new_q}, {new_normal});
  {
    const std::vector<VectorXd>& q0 = engine.world_configurations(id0);
    ASSERT_EQ(q0.size(), 1);
    EXPECT_EQ(q0[0], new_q);
    const std::vector<VectorXd>& q1 = engine.world_configurations(id1);
    ASSERT_EQ(q1.size(), 2);
    EXPECT_EQ(q1[0], initial_q);
    EXPECT_EQ(q1[1], initial_q);
  }
  {
    const std::vector<VectorXd>& n0 = engine.world_normals(id0);
    ASSERT_EQ(n0.size(), 1);
    EXPECT_EQ(n0[0], new_normal);
    const std::vector<VectorXd>& n1 = engine.world_normals(id1);
    ASSERT_EQ(n1.size(), 2);
    EXPECT_EQ(n1[0], initial_normal);
    EXPECT_EQ(n1[1], initial_normal);
  }

  // Now update id1 to verify that updates to a multi-mesh geometry go to the
  // right meshes.
  VectorXd another_q(3);
  VectorXd another_normal(3);
  another_q << 40, 50, 60;
  another_normal << 70, 80, 90;
  another_normal.normalize();
  engine.UpdateDeformableConfigurations(id1, {new_q, another_q},
                                        {new_normal, another_normal});
  {
    const std::vector<VectorXd>& q1 = engine.world_configurations(id1);
    ASSERT_EQ(q1.size(), 2);
    EXPECT_EQ(q1[0], new_q);
    EXPECT_EQ(q1[1], another_q);
    const std::vector<VectorXd>& n1 = engine.world_normals(id1);
    ASSERT_EQ(n1.size(), 2);
    EXPECT_EQ(n1[0], new_normal);
    EXPECT_EQ(n1[1], another_normal);
  }

  // Now we test for throw conditions for the update.
  // Non-existent geometry is fine. It's silently ignored.
  const GeometryId fake_id = GeometryId::get_new_id();
  EXPECT_NO_THROW(
      engine.UpdateDeformableConfigurations(fake_id, {new_q}, {new_normal}));
  // Wrong number of vertex positions/normals.
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.UpdateDeformableConfigurations(id0, {new_q, new_q}, {new_normal}),
      "Vertex data for the wrong number of meshes.*1 meshes are "
      "registered.*vertex positions for 2 meshes and vertex normals for 1 "
      "meshes are provided.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.UpdateDeformableConfigurations(id0, {new_q},
                                            {new_normal, new_normal}),
      "Vertex data for the wrong number of meshes.*1 meshes are "
      "registered.*vertex positions for 1 meshes and vertex normals for 2 "
      "meshes are provided.*");
  // Wrong size for the vertex positions/normal vectors.
  VectorXd incorrectly_sized_vector(4);
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.UpdateDeformableConfigurations(id0, {incorrectly_sized_vector},
                                            {new_normal}),
      "Wrong dofs in vertex positions and/or normals.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.UpdateDeformableConfigurations(id0, {new_q},
                                            {incorrectly_sized_vector}),
      "Wrong dofs in vertex positions and/or normals.*");
}

// Tests the RenderEngine-specific functionality for managing registration of
// rigid geometry and its corresponding update behavior. The former should
// configure each geometry correctly on whether it gets updated or not, and the
// latter will confirm that the right geometries get updated.
GTEST_TEST(RenderEngine, RigidGeometryRegistrationAndUpdate) {
  // Change the default render label to something registerable.
  DummyRenderEngine engine({RenderLabel::kDontCare});

  // Configure parameters for registering visuals.
  PerceptionProperties skip_properties = engine.rejecting_properties();
  PerceptionProperties add_properties = engine.accepting_properties();
  Sphere sphere(1.0);

  // A collection of poses to provide to calls to UpdatePoses(). Configured
  // to all identity transforms because the values generally don't matter. In
  // the single case where it does matter, a value is explicitly set (see
  // below).
  unordered_map<GeometryId, RigidTransformd> X_WG_all{
      {GeometryId::get_new_id(), RigidTransformd{Vector3d{1, 2, 3}}},
      {GeometryId::get_new_id(), RigidTransformd{Vector3d{2, 3, 4}}},
      {GeometryId::get_new_id(), RigidTransformd{Vector3d{3, 4, 5}}}};

  // These test cases are accumulative; re-ordering them will require
  // refactoring.

  // Tests that rely on the RenderEngine's default value are tested below in the
  // DefaultRenderLabel test.

  // Create properties with the given RenderLabel value. Used only to test
  // failure when providing an invalid render label. (See Case 1.)
  auto make_properties = [](RenderLabel label) {
    PerceptionProperties properties;
    properties.AddProperty("label", "id", label);
    return properties;
  };

  // Case 1: Explicitly providing the unspecified or empty render label throws.
  for (const auto& label : {RenderLabel::kEmpty, RenderLabel::kUnspecified}) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        engine.RegisterVisual(GeometryId::get_new_id(), sphere,
                              make_properties(label), {}, false),
        "Cannot register a geometry with the 'unspecified' or 'empty' render "
        "labels.*");
  }

  // Case: the shape is configured to be ignored by the render engine. Returns
  // false (and other arguments do not matter).
  const GeometryId id1 = GeometryId::get_new_id();
  const bool is_dynamic = true;
  const bool dynamic_accepted =
      engine.RegisterVisual(id1, sphere, skip_properties, {}, is_dynamic);
  EXPECT_FALSE(dynamic_accepted);
  const bool anchored_accepted =
      engine.RegisterVisual(id1, sphere, skip_properties, {}, !is_dynamic);
  EXPECT_FALSE(anchored_accepted);
  // Confirm nothing is updated - because nothing is registered.
  engine.UpdatePoses(X_WG_all);
  EXPECT_EQ(engine.updated_ids().size(), 0);

  {
    // Case: the shape is configured for registration, but does *not* require
    // updating.
    const auto& [id, X_WG] = *(X_WG_all.begin());
    bool accepted =
        engine.RegisterVisual(id, sphere, add_properties, X_WG, !is_dynamic);
    EXPECT_TRUE(accepted);
    EXPECT_TRUE(CompareMatrices(engine.world_pose(id).GetAsMatrix34(),
                                X_WG.GetAsMatrix34()));
    engine.UpdatePoses(X_WG_all);
    EXPECT_EQ(engine.updated_ids().size(), 0);
    EXPECT_TRUE(CompareMatrices(engine.world_pose(id).GetAsMatrix34(),
                                X_WG.GetAsMatrix34()));
  }

  {
    // Case: the shape is configured for registration *and* requires updating.
    // Configure the pose for the id so it is *not* the identity.
    const auto& [id, X_WG] = *(++(X_WG_all.begin()));
    bool accepted =
        engine.RegisterVisual(id, sphere, add_properties, X_WG, is_dynamic);
    EXPECT_TRUE(accepted);
    const Vector3d p_WG(1.5, 2.5, 3.5);
    X_WG_all[id].set_translation(p_WG);
    engine.UpdatePoses(X_WG_all);
    EXPECT_EQ(engine.updated_ids().size(), 1);
    ASSERT_TRUE(engine.updated_ids().contains(id));
    EXPECT_TRUE(
        CompareMatrices(engine.updated_ids().at(id).translation(), p_WG));
    EXPECT_TRUE(CompareMatrices(engine.world_pose(id).GetAsMatrix34(),
                                X_WG_all[id].GetAsMatrix34()));
  }
}

// Tests the removal of geometry from the renderer -- confirms that the
// RenderEngine removes the geometry appropriately.
GTEST_TEST(RenderEngine, RemoveGeometry) {
  const int need_update_count = 3;
  const int anchored_count = 2;

  // The test render engine contains three dynamic geometries and two anchored.

  // Set the default render label to something registerable.
  DummyRenderEngine engine({RenderLabel::kDontCare});
  // A set of properties that will cause a shape to be properly registered.
  PerceptionProperties add_properties = engine.accepting_properties();
  RigidTransformd X_WG = RigidTransformd::Identity();
  Sphere sphere(1.0);

  set<GeometryId> ids;

  // Register rigid geometries.
  for (int i = 0; i < need_update_count + anchored_count; ++i) {
    const GeometryId id = GeometryId::get_new_id();
    const bool is_dynamic = i % 2 == 0;  // alternate dynamic, anchored, etc.
    const bool accepted =
        engine.RegisterVisual(id, sphere, add_properties, X_WG, is_dynamic);
    if (!accepted) {
      throw std::logic_error("The geometry wasn't accepted for registration");
    }
    ids.insert(id);
  }

  // Register a single deformable geometry.
  geometry::internal::RenderMesh mesh;
  GeometryId deformable_id = GeometryId::get_new_id();
  const bool accepted =
      engine.RegisterDeformableVisual(deformable_id, {mesh}, add_properties);
  if (!accepted) {
    throw std::logic_error("The geometry wasn't accepted for registration");
  }
  ids.insert(deformable_id);

  RenderEngineTester tester(&engine);

  // Case: invalid ids don't get removed.
  EXPECT_FALSE(engine.RemoveGeometry(GeometryId::get_new_id()));

  // Case: Systematically remove remaining geometries and confirm state --
  // because of how geometries were added, this will alternate dynamic,
  // anchored, dynamic, etc, with the last geometry being deformable.
  while (!ids.empty()) {
    const GeometryId id = *ids.begin();
    EXPECT_TRUE(engine.RemoveGeometry(id));
    ids.erase(ids.begin());
    EXPECT_EQ(static_cast<int>(ids.size()), tester.num_geometries());
    for (GeometryId remaining_id : ids) {
      EXPECT_TRUE(tester.has_id(remaining_id));
    }
  }
}

GTEST_TEST(RenderEngine, ColorLabelConversion) {
  // Explicitly testing labels at *both* ends of the reserved space -- this
  // assumes that the reserved labels are at the top end; if that changes, we'll
  // need a different mechanism to get a large-valued label.
  const std::array<RenderLabel, 3> labels{
      RenderLabel(0),
      RenderLabel(RenderLabel::kMaxUnreserved - 1),
      RenderLabel::kEmpty,
  };
  for (size_t i = 0; i < labels.size(); ++i) {
    for (size_t j = 0; j < labels.size(); ++j) {
      if (i == j) {
        // Colors should be invertible back to the original label.
        const Rgba color = DummyRenderEngine::MakeRgbFromLabel(labels[i]);
        const Vector3<uint8_t> pixel =
            (color.rgba().head(3) * 255.0).template cast<uint8_t>();
        const RenderLabel readback =
            DummyRenderEngine::MakeLabelFromRgb(pixel[0], pixel[1], pixel[2]);
        EXPECT_EQ(readback, labels[i]);
      } else {
        // Different labels should produce different colors.
        EXPECT_NE(DummyRenderEngine::MakeRgbFromLabel(labels[i]),
                  DummyRenderEngine::MakeRgbFromLabel(labels[j]));
      }
    }
  }
}

// Tests the documented behavior for configuring the default render label.
GTEST_TEST(RenderEngine, DefaultRenderLabel) {
  // Case: Confirm RenderEngine default is kUnspecified.
  {
    DummyRenderEngine engine;
    EXPECT_EQ(engine.default_render_label(), RenderLabel::kUnspecified);
  }

  // Case: Confirm kDontCare is valid.
  {
    DummyRenderEngine engine(RenderLabel::kDontCare);
    EXPECT_EQ(engine.default_render_label(), RenderLabel::kDontCare);
  }

  // Case: Confirm construction with alternate label is forbidden.
  {
    for (auto label :
         {RenderLabel::kDoNotRender, RenderLabel::kEmpty, RenderLabel{10}}) {
      DRAKE_EXPECT_THROWS_MESSAGE(DummyRenderEngine{label},
                                  ".* default render label must be either "
                                  "'kUnspecified' or 'kDontCare'");
    }
  }
}

// The render API with full intrinsics promises some validation before calling
// the virtual DoRender*Image() API. Confirm that it happens.
GTEST_TEST(RenderEngine, ValidateIntrinsicsAndImage) {
  const DummyRenderEngine engine;
  const int w = 2;
  const int h = 2;
  const CameraInfo intrinsics{w, h, M_PI};
  const ColorRenderCamera color_camera{
      {"n/a", intrinsics, {0.1, 10}, RigidTransformd{}}, false};
  const DepthRenderCamera depth_camera{
      {"n/a", intrinsics, {0.1, 10}, RigidTransformd{}}, {1.0, 5.0}};

  // Image is nullptr.
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.RenderColorImage(color_camera, nullptr),
      "Can't render a color image. The given output image is nullptr");
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.RenderDepthImage(depth_camera, nullptr),
      "Can't render a depth image. The given output image is nullptr");
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.RenderLabelImage(color_camera, nullptr),
      "Can't render a label image. The given output image is nullptr");

  // Image size doesn't match.
  constexpr const char* error_message =
      "The {} image to write has a size different from that specified in "
      "the camera intrinsics. Image: \\({}, {}\\), intrinsics: \\({}, {}\\)";

  auto test_bad_sizes = [&](const char* image_type, auto image_example,
                            auto render) {
    using Image = decltype(image_example);
    std::vector<Image> images{Image{w - 1, h}, Image{w, h - 1},
                              Image{w - 1, h - 1}};
    for (Image& i : images) {
      DRAKE_EXPECT_THROWS_MESSAGE(
          render(&i),
          fmt::format(error_message, image_type, i.width(), i.height(),
                      intrinsics.width(), intrinsics.height()));
    }
  };

  test_bad_sizes("color", ImageRgba8U{1, 1}, [&](ImageRgba8U* i) {
    engine.RenderColorImage(color_camera, i);
  });

  test_bad_sizes("depth", ImageDepth32F{1, 1}, [&](ImageDepth32F* i) {
    engine.RenderDepthImage(depth_camera, i);
  });

  test_bad_sizes("label", ImageLabel16I{1, 1}, [&](ImageLabel16I* i) {
    engine.RenderLabelImage(color_camera, i);
  });
}

// An absolute barebones RenderEngine implementation; however it is cloneable
// with both a copy constructor *and* a valid DoClone() implementation.
class CloneableEngine : public MinimumEngine {
 public:
  CloneableEngine(const CloneableEngine&) = default;
  CloneableEngine& operator=(const CloneableEngine&) = delete;
  CloneableEngine(CloneableEngine&&) = delete;
  CloneableEngine& operator=(CloneableEngine&&) = delete;

  CloneableEngine() = default;

  std::unique_ptr<RenderEngine> DoClone() const override {
    return std::make_unique<CloneableEngine>(*this);
  }
};

// A derivative of CloneableEngine that has forgotten to implement DoClone().
// That should produce an error.
class NoDoCloneEngine : public CloneableEngine {
 public:
  NoDoCloneEngine() = default;
};

// Confirms that the guard to detect a derived RenderEngine that forgot to
// implement DoClone().
GTEST_TEST(RenderEngine, DetectDoCloneFailure) {
  CloneableEngine cloneable;
  EXPECT_NO_THROW(cloneable.Clone());
  EXPECT_NO_THROW(cloneable.Clone<std::unique_ptr<RenderEngine>>());
  EXPECT_NO_THROW(cloneable.Clone<std::shared_ptr<RenderEngine>>());

  NoDoCloneEngine not_cloneable;
  DRAKE_EXPECT_THROWS_MESSAGE(
      not_cloneable.Clone(),
      "Error in cloning RenderEngine class of type .+NoDoCloneEngine; the "
      "clone returns type .+CloneableEngine. .+NoDoCloneEngine::DoClone.. was "
      "probably not implemented");
  DRAKE_EXPECT_THROWS_MESSAGE(
      not_cloneable.Clone<std::unique_ptr<RenderEngine>>(),
      ".*not implemented");
  DRAKE_EXPECT_THROWS_MESSAGE(
      not_cloneable.Clone<std::shared_ptr<RenderEngine>>(),
      ".*not implemented");
}

class CloneableAsSharedOnlyEngine final : public RenderEngine {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CloneableAsSharedOnlyEngine);
  CloneableAsSharedOnlyEngine() = default;

  // Only implement shared_ptr cloning.
  std::unique_ptr<RenderEngine> DoClone() const final {
    throw std::logic_error("SHARED ONLY");
  }
  std::shared_ptr<RenderEngine> DoCloneShared() const final {
    return std::make_unique<CloneableAsSharedOnlyEngine>(*this);
  }

  // No-op fluff for the pure virtuals.
  void UpdateViewpoint(const math::RigidTransformd&) final {}
  bool DoRegisterVisual(GeometryId, const Shape&, const PerceptionProperties&,
                        const RigidTransformd&) final {
    return false;
  }
  void DoUpdateVisualPose(GeometryId, const math::RigidTransformd&) final {}
  bool DoRemoveGeometry(GeometryId) final { return false; }
};

GTEST_TEST(RenderEngine, CloneOnlySupportsShared) {
  CloneableAsSharedOnlyEngine dut;
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Clone(), "SHARED ONLY");
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Clone<std::unique_ptr<RenderEngine>>(),
                              "SHARED ONLY");
  EXPECT_NO_THROW(dut.Clone<std::shared_ptr<RenderEngine>>());
}

// Confirms that sub-classes that don't implement the DoRender*Image API get
// unimplemented exceptions.
GTEST_TEST(RenderEngine, UnimplementedRenderErrors) {
  MinimumEngine engine;
  const int w = 2;
  const int h = 2;
  const CameraInfo intrinsics{w, h, M_PI};
  const ColorRenderCamera color_camera{
      {"n/a", intrinsics, {0.1, 10}, RigidTransformd{}}, false};
  const DepthRenderCamera depth_camera{
      {"n/a", intrinsics, {0.1, 10}, RigidTransformd{}}, {1.0, 5.0}};

  ImageRgba8U color{w, h};
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.RenderColorImage(color_camera, &color),
      ".*MinimumEngine.* has not implemented DoRenderColorImage.+");
  ImageDepth32F depth{w, h};
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.RenderDepthImage(depth_camera, &depth),
      ".*MinimumEngine.* has not implemented DoRenderDepthImage.+");
  ImageLabel16I label{w, h};
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.RenderLabelImage(color_camera, &label),
      ".*MinimumEngine.* has not implemented DoRenderLabelImage.+");
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake

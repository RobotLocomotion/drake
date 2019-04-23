#include "drake/geometry/render/render_engine.h"

#include <map>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/drake_optional.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace render {

using math::RigidTransformd;

class RenderEngineTester {
 public:
  explicit RenderEngineTester(const RenderEngine* engine) : engine_(*engine) {}

  const std::unordered_map<RenderIndex, GeometryIndex>& update_map() const {
    return engine_.update_indices_;
  }

  const std::unordered_map<RenderIndex, GeometryIndex>& anchored_map() const {
    return engine_.anchored_indices_;
  }

 private:
  const RenderEngine& engine_;
};

namespace {

// Dummy implementation of the RenderEngine interface to facilitate testing
// the specific RenderEngine functionality.
class DummyRenderEngine final : public RenderEngine {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DummyRenderEngine);
  DummyRenderEngine() = default;
  void UpdateViewpoint(const RigidTransformd&) const final {}
  void RenderColorImage(const render::CameraProperties&, bool,
                        systems::sensors::ImageRgba8U*) const final {}
  void RenderDepthImage(const render::DepthCameraProperties&,
                        systems::sensors::ImageDepth32F*) const final {}
  void ImplementGeometry(const Sphere& sphere, void* user_data) final {}
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) final {}
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) final {}
  void ImplementGeometry(const Box& box, void* user_data) final {}
  void ImplementGeometry(const Mesh& mesh, void* user_data) final {}
  void ImplementGeometry(const Convex& convex, void* user_data) final {}

  // Test harness.

  // Provide access to the group name that implies full registration.
  const std::string& include_group_name() const { return include_group_name_; }

  // Return the indices that have been updated via a call to UpdatePoses().
  const std::map<RenderIndex, RigidTransformd>& updated_indices() const {
    return updated_indices_;
  }

  // Utility to set what value DoRemoveGeometry() returns to facilitate testing.
  void set_moved_index(optional<RenderIndex> index) {
    moved_index_ = index;
  }

 protected:
  // Conditionally register the visual based on the properties having an
  // "in_test" group.
  optional<RenderIndex> DoRegisterVisual(const Shape&,
                                         const PerceptionProperties& properties,
                                         const RigidTransformd&) final {
    if (properties.HasGroup(include_group_name_)) {
      return RenderIndex(register_count_++);
    }
    return nullopt;
  }

  // Track all of the indices provided an update pose for.
  void DoUpdateVisualPose(RenderIndex index,
                          const RigidTransformd& X_WG) final {
    updated_indices_[index] = X_WG;
  }

  optional<RenderIndex> DoRemoveGeometry(RenderIndex index) final {
    return moved_index_;
  }

  std::unique_ptr<render::RenderEngine> DoClone() const final {
    return std::make_unique<DummyRenderEngine>(*this);
  }

 private:
  int register_count_{};
  // Track each index and what it has been updated to (allows us to confirm
  // RenderIndex and GeometryIndex association.
  std::map<RenderIndex, RigidTransformd> updated_indices_;
  // The group name whose presence will lead to a shape being added to the
  // engine.
  std::string include_group_name_{"in_test"};
  // The RenderIndex value to return on invocation of DoRemoveGeometry().
  optional<RenderIndex> moved_index_;
};

// Tests the RenderEngine-specific functionality for managing registration of
// geometry and its corresponding update behavior. The former should configure
// each geometry correctly on whether it gets updated or not, and the latter
// will confirm that the right geometries get updated.
GTEST_TEST(RenderEngine, RegistrationAndUpdate) {
  DummyRenderEngine engine;

  // Configure parameters for registering visuals.
  PerceptionProperties skip_properties;
  PerceptionProperties add_properties;
  add_properties.AddProperty(engine.include_group_name(), "ignored", 0);
  Sphere sphere(1.0);
  RigidTransformd X_WG = RigidTransformd::Identity();
  // A collection of poses to provide to calls to UpdatePoses(). Configured
  // to all identity transforms because the values generally don't matter. In
  // the single case where it does matter, a value is explicitly set (see
  // below).
  std::vector<RigidTransformd> X_WG_all{3, X_WG};

  // These test cases are accumulative; re-ordering them will require
  // refactoring.

  // Case: the shape is configured to be ignored by the render engine. Returns
  // nullopt (and other arguments do not matter).
  optional<RenderIndex> optional_index = engine.RegisterVisual(
      GeometryIndex(0), sphere, skip_properties, X_WG, false);
  EXPECT_FALSE(optional_index);
  optional_index = engine.RegisterVisual(GeometryIndex(0), sphere,
                                         skip_properties, X_WG, true);
  EXPECT_FALSE(optional_index);
  // Confirm nothing is updated - because nothing is registered.
  engine.UpdatePoses(X_WG_all);
  EXPECT_EQ(engine.updated_indices().size(), 0);

  // Case: the shape is configured for registration, but does *not* require
  // updating. We get a valid render index, but it is _not_ included in
  // UpdatePoses().
  optional_index = engine.RegisterVisual(GeometryIndex(1), sphere,
                                         add_properties, X_WG, false);
  EXPECT_TRUE(optional_index);
  engine.UpdatePoses(X_WG_all);
  EXPECT_EQ(engine.updated_indices().size(), 0);

  // Case: the shape is configured for registration *and* requires updating. We
  // get a valid render index and it _is_ included in UpdatePoses().
  GeometryIndex update_index{2};
  // Configure the pose for index 2 to *not* be the identity so we can confirm
  // that the registered GeometryIndex is properly associated with the resulting
  // RenderIndex.
  const Vector3<double> p_WG(1, 2, 3);
  X_WG_all[update_index].set_translation(p_WG);
  optional_index =
      engine.RegisterVisual(update_index, sphere, add_properties, X_WG, true);
  EXPECT_TRUE(optional_index);
  engine.UpdatePoses(X_WG_all);
  EXPECT_EQ(engine.updated_indices().size(), 1);
  ASSERT_EQ(engine.updated_indices().count(*optional_index), 1);
  EXPECT_TRUE(CompareMatrices(
      engine.updated_indices().at(*optional_index).translation(), p_WG));
}

// Tests the removal of geometry from the renderer -- confirms that the
// RenderEngine is
//   a) Reporting the correct geometry index for the removed geometry and
//   b) Updating the remaining RenderIndex -> GeometryIndex pairs correctly.
GTEST_TEST(RenderEngine, RemoveGeometry) {
  const int need_update_count = 3;
  const int anchored_count = 2;
  std::vector<GeometryIndex> render_index_to_geometry_index;
  // Configure a clean render engine so each test is independent. Specifically,
  // It creates three dynamic geometries and two anchored. The initial render
  // index and geometry index matches for each geometry. Conceptually, we'll
  // have two maps:
  //  dynamic map: {{0, 0}, {1, 1}, {2, 2}}
  //  anchored map: {{3, 3}, {4, 4}}
  // Ultimately, we'll examine the the maps after removing geometry to confirm
  // the state of the mappings as a perturbation from this initial condition.
  auto make_engine = [&render_index_to_geometry_index]() -> DummyRenderEngine {
    render_index_to_geometry_index.clear();
    DummyRenderEngine engine;
    // A set of properties that will cause a shape to be properly registered.
    PerceptionProperties add_properties;
    add_properties.AddProperty(engine.include_group_name(), "ignored", 0);
    RigidTransformd X_WG = RigidTransformd::Identity();
    Sphere sphere(1.0);

    for (int i = 0; i < need_update_count + anchored_count; ++i) {
      const GeometryIndex geometry_index = GeometryIndex(i);
      optional<RenderIndex> render_index = engine.RegisterVisual(
          geometry_index, sphere, add_properties, X_WG, i < need_update_count);
      if (!render_index || *render_index != i) {
        throw std::logic_error("Unexpected render indices");
      }
      render_index_to_geometry_index.push_back(geometry_index);
    }
    return engine;
  };

  // Function for performing the removal and testing the results.
  auto expect_removal =
      [make_engine](RenderIndex remove_index, optional<RenderIndex> move_index,
         optional<GeometryIndex> expected_moved,
         std::initializer_list<std::pair<int, int>> dynamic_map,
         std::initializer_list<std::pair<int, int>> anchored_map,
         const char* case_description) {
        DummyRenderEngine engine = make_engine();
        engine.set_moved_index(move_index);
        optional<GeometryIndex> moved_index =
            engine.RemoveGeometry(remove_index);
        EXPECT_EQ(moved_index, expected_moved) << case_description;
        RenderEngineTester tester(&engine);
        EXPECT_EQ(tester.update_map().size(), dynamic_map.size())
            << case_description;
        for (const auto& pair : dynamic_map) {
          EXPECT_EQ(tester.update_map().at(RenderIndex(pair.first)),
                    GeometryIndex(pair.second))
              << case_description;
        }
        EXPECT_EQ(tester.anchored_map().size(), anchored_map.size())
            << case_description;
        for (const auto& pair : anchored_map) {
          EXPECT_EQ(tester.anchored_map().at(RenderIndex(pair.first)),
                    GeometryIndex(pair.second))
              << case_description;
        }
      };

  using IndexMap = std::initializer_list<std::pair<int, int>>;

  // Case 1: remove dynamic geometry where nothing gets moved. Specifically,
  // remove the geometry (2, 2) with nothing else changing.
  {
    const RenderIndex remove_index(need_update_count - 1);
    optional<RenderIndex> moved_render_index = nullopt;
    optional<GeometryIndex> moved_geometry_index = nullopt;
    // Note: loss of need_update_count - 1 (i.e., 2) from the dynamic map.
    IndexMap expected_dynamic_map = {{0, 0}, {1, 1}};
    IndexMap expected_anchored_map = {{3, 3}, {4, 4}};
    expect_removal(remove_index, moved_render_index, moved_geometry_index,
                   expected_dynamic_map, expected_anchored_map, "case 1");
  }

  // Case 2: remove dynamic geometry where dynamic geometry gets moved.
  // Specifically, we have three dynamic geometries (0, 1, 2). We remove
  // geometry 1 and 2 gets moved into its slot.
  {
    const RenderIndex remove_index(1);
    const int move_value = 2;
    optional<RenderIndex> moved_render_index = RenderIndex(move_value);
    optional<GeometryIndex> moved_geometry_index = GeometryIndex(move_value);
    // Note: loss (1, 1) and (2, 2) gets moved to (1, 2) in the dynamic map.
    IndexMap expected_dynamic_map = {{0, 0}, {1, 2}};
    IndexMap expected_anchored_map = {{3, 3}, {4, 4}};
    expect_removal(remove_index, moved_render_index, moved_geometry_index,
                   expected_dynamic_map, expected_anchored_map, "case 2");
  }

  // Case 3: remove dynamic geometry where anchored geometry gets moved.
  // Specifically, remove the last dynamic geometry (2, 2), and move the last
  // anchored geometry (4, 4) into its slot.
  {
    const RenderIndex remove_index(2);
    const int move_value = 4;
    optional<RenderIndex> moved_render_index = RenderIndex(move_value);
    optional<GeometryIndex> moved_geometry_index = GeometryIndex(move_value);
    // Note: loss of (2, 2) from the dynamic map.
    IndexMap expected_dynamic_map = {{0, 0}, {1, 1}};
    // Note: (4, 4) gets moved to use the removed render index (2). So,
    // (4, 4) becomes (2, 4).
    IndexMap expected_anchored_map = {{3, 3}, {2, 4}};
    expect_removal(remove_index, moved_render_index, moved_geometry_index,
                   expected_dynamic_map, expected_anchored_map, "case 3");
  }

  // Case 4: remove anchored geometry where nothing gets moved.
  {
    const RenderIndex remove_index(4);
    optional<RenderIndex> moved_render_index = nullopt;
    optional<GeometryIndex> moved_geometry_index = nullopt;
    // Dynamic map untouched.
    IndexMap expected_dynamic_map = {{0, 0}, {1, 1}, {2, 2}};
    // Geometry (4, 4) has been removed.
    IndexMap expected_anchored_map = {{3, 3}};
    expect_removal(remove_index, moved_render_index, moved_geometry_index,
                   expected_dynamic_map, expected_anchored_map, "case 4");
  }

  // Case 5: remove anchored geometry where dynamic geometry gets moved.
  // Specifically, remove the _last_ anchored geometry (4, 4) and move the
  // last dynamic geometry (2, 2) into that slot to become (2, 4).
  {
    const RenderIndex remove_index(4);
    const int move_value = 2;
    optional<RenderIndex> moved_render_index = RenderIndex(move_value);
    optional<GeometryIndex> moved_geometry_index = GeometryIndex(move_value);
    // Note: (2, 2) has been moved into the remove index (4, 2).
    IndexMap expected_dynamic_map = {{0, 0}, {1, 1}, {4, 2}};
    // Note: (4, 4) has been removed
    IndexMap expected_anchored_map = {{3, 3}};
    expect_removal(remove_index, moved_render_index, moved_geometry_index,
                   expected_dynamic_map, expected_anchored_map, "case 5");
  }

  // Case 6: remove anchored geometry where anchored geometry gets moved.
  // We have two anchored geometries: (3, 3) and (4, 4). Remove (3, 3) and move
  // (4, 4) into its place (3, 4).
  {
    const RenderIndex remove_index(3);
    const int move_value = 4;
    optional<RenderIndex> moved_render_index = RenderIndex(move_value);
    optional<GeometryIndex> moved_geometry_index = GeometryIndex(move_value);
    // Note: Dynamic is untouched.
    IndexMap expected_dynamic_map = {{0, 0}, {1, 1}, {2, 2}};
    // Note: Remove (3, 3) and move (4, 4) into position 3: (3, 4).
    IndexMap expected_anchored_map = {{3, 4}};
    expect_removal(remove_index, moved_render_index, moved_geometry_index,
                   expected_dynamic_map, expected_anchored_map, "case 6");
  }
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake

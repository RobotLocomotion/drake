// This file is derived from the python prototype at
// https://github.com/EricCousineau-TRI/repro/blob/c44615ee725b9c5498cd5b1d706ccfec6c3bd203/drake_stuff/multibody_plant_prototypes/multibody_plant_subgraph_test.py.
#include "drake/multibody/parsing/detail_multibody_plant_subgraph.h"

#include <cstddef>
#include <iostream>
#include <random>
#include <tuple>
#include <typeinfo>
#include <variant>

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <gflags/gflags.h>
#include <gmock/gmock.h>
#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/universal_joint.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_bool(visualize, false,
            "If true, visualize simulation using DrakeVisualizer.");

namespace drake {
namespace multibody {
namespace internal {
namespace {

using geometry::Box;
using geometry::DrakeVisualizerd;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::HalfSpace;
using geometry::SceneGraph;
using math::RigidTransformd;
using systems::Context;
using systems::DiagramBuilder;
using systems::Simulator;

struct MultibodyPlantElementsCount {
  template <typename T>
  explicit MultibodyPlantElementsCount(const T& elem)
      : model_instances(elem.model_instances().size()),
        bodies(elem.bodies().size()),
        frames(elem.frames().size()),
        joints(elem.joints().size()),
        joint_actuators(elem.joint_actuators().size()),
        geometry_ids(elem.geometry_ids().size()) {}

  MultibodyPlantElementsCount(std::size_t _model_instances, std::size_t _bodies,
                              std::size_t _frames, std::size_t _joints,
                              std::size_t _joint_actuators,
                              std::size_t _geometry_ids)
      : model_instances(_model_instances),
        bodies(_bodies),
        frames(_frames),
        joints(_joints),
        joint_actuators(_joint_actuators),
        geometry_ids(_geometry_ids) {}

  std::size_t model_instances;
  std::size_t bodies;
  std::size_t frames;
  std::size_t joints;
  std::size_t joint_actuators;
  std::size_t geometry_ids;
};

// Used in EXPECT_* calls
std::ostream& operator<<(std::ostream& os,
    MultibodyPlantElementsCount c) {
  os << fmt::format(
      "model_instances: {}, bodies: {}, frames: {}, joints: {}, "
      "joint_actuators: {}, geometry_ids: {}",
      c.model_instances, c.bodies, c.frames, c.joints, c.joint_actuators,
      c.geometry_ids);
  return os;
}

bool operator==(const MultibodyPlantElementsCount& a,
                const MultibodyPlantElementsCount& b) {
  return a.model_instances == b.model_instances && a.bodies == b.bodies &&
         a.frames == b.frames && a.joints == b.joints &&
         a.joint_actuators == b.joint_actuators &&
         a.geometry_ids == b.geometry_ids;
}

// Helper struct that is returned by CastTo defined below.
template <typename... C>
struct CastResult {
  template <typename T>
  explicit CastResult(const T* obj) : cast_objs(dynamic_cast<C>(obj)...) {}

  explicit operator bool() {
    return ((std::get<C>(cast_objs) != nullptr) && ...);
  }

  template <typename F>
  void DoIfCastable(F cb) {
    auto visitor = [&cb](const auto* arg) {
      if (arg != nullptr) {
        cb(arg);
      }
    };
    (std::invoke(visitor, std::get<C>(cast_objs)), ...);
  }

  std::tuple<C...> cast_objs;
};

// Helper function that checks if @p obj can be dynamically cast to one of the
// types in @p C. If so, the returned CastResult object will be truthy and one
// can call CastResult::DoIfCastable with a callback function that gets invoked
// for every type that casts successfully.
template <typename... C, typename T>
CastResult<C...> CastsTo(const T* obj) {
  return CastResult<C...>(obj);
}
// TODO(azeey) Replace with std::ranges when the feature is available.
template <typename T, typename S> struct Zip {
  Zip(T &a, S &b) : a_(a), b_(b) {}

  class Iterator
      : public std::iterator<std::input_iterator_tag, std::pair<T, S>> {
   public:
    using reference =
        std::pair<decltype(*T().begin())&, decltype(*S().begin())&>;
    using IteratorTypeT = decltype(T().begin());
    using IteratorTypeS = decltype(S().begin());

    Iterator(IteratorTypeT a, IteratorTypeS b) : a_(a), b_(b) {}
    Iterator& operator++() {
      ++a_;
      ++b_;
      return *this;
    }

    bool operator==(Iterator other) const {
      return a_ == other.a_ && b_ == other.b_;
    }
    bool operator!=(Iterator other) const { return !(*this == other); }
    reference operator*() const { return {*a_, *b_}; }

   private:
    IteratorTypeT a_;
    IteratorTypeS b_;
  };

  Iterator begin() { return Iterator(a_.begin(), b_.begin()); }
  Iterator end() { return Iterator(a_.end(), b_.end()); }

  T &a_;
  S &b_;
};

// Checks that two multibody elements have similar base properties.
template <typename T>
void CheckElement(const T& a, const T& b, bool check_index = true) {
  EXPECT_NE(&a, &b);
  if (check_index) {
    EXPECT_EQ(a.index(), b.index()) << fmt::format(
        "{}({}) != {}({})", a.name(), a.index(), b.name(), b.index());
  }
  EXPECT_EQ(a.name(), b.name());
  EXPECT_EQ(a.model_instance(), b.model_instance());
}

void AssertShapeEquals(const geometry::Shape& a, const geometry::Shape& b) {
  EXPECT_EQ(typeid(a), typeid(b));
  if (auto resultBox = CastsTo<const Box*>(&a)) {
    resultBox.DoIfCastable([&b](const auto* box_a) {
      const Box* box_b = dynamic_cast<const Box*>(&b);
      ASSERT_NE(box_b, nullptr);
      EXPECT_EQ(box_a->width(), box_b->width());
      EXPECT_EQ(box_a->height(), box_b->height());
      EXPECT_EQ(box_a->depth(), box_b->depth());
    });
  } else if (CastsTo<const HalfSpace*>(&a)) {
    // Do nothing
  } else {
    GTEST_FAIL();
  }
}

void AssertValueEquals(const drake::AbstractValue& value_a,
    const drake::AbstractValue& value_b) {
  ASSERT_EQ(value_a.type_info(), value_b.type_info());
  if (const auto* a = value_a.maybe_get_value<CoulombFriction<double>>()) {
    auto *b = value_b.maybe_get_value<CoulombFriction<double>>();
    ASSERT_NE(b, nullptr);
    EXPECT_EQ(a->static_friction(), b->static_friction());
    EXPECT_EQ(a->dynamic_friction(), b->dynamic_friction());
  } else {
    // TODO(azeey) The python prototype compares the values of a and b, but we
    // can't do that in C++ without knowing the concrete types represented by
    // the abstract values.
  }
}

void AssertPropertiesEquals(const geometry::GeometryProperties* prop_a,
    const geometry::GeometryProperties* prop_b) {
  if (prop_a == nullptr) {
    ASSERT_EQ(prop_b, nullptr);
    return;
  }

  const auto& groups = prop_a->GetGroupNames();
  ASSERT_EQ(groups, prop_b->GetGroupNames());

  for (const auto& group_name : groups) {
    const auto& group_a = prop_a->GetPropertiesInGroup(group_name);
    const auto& group_b = prop_b->GetPropertiesInGroup(group_name);
    ASSERT_EQ(group_a.size(), group_b.size());
    for (const auto& [name, value_a] : group_a) {
      const auto& value_b = group_b.at(name);
      AssertValueEquals(*value_a, *value_b);
    }
  }
}

// Asserts that two plants are (almost) completely equal; more specifically:
// - All model instances, bodies, joints, and joint actuators have the same
//   indices.
//     - Frames may have different indices, due to ordering.
// - All properties of each element are "exactly" the same.
void AssertPlantEquals(const MultibodyPlant<double>* plant_a,
                       const SceneGraph<double>* scene_graph_a,
                       const MultibodyPlant<double>* plant_b,
                       const SceneGraph<double>* scene_graph_b) {
  ASSERT_NE(&plant_a, &plant_b);
  if (scene_graph_a != nullptr) {
    ASSERT_NE(scene_graph_b, nullptr);
  }
  auto elem_a = MultibodyPlantElements::FromPlant(plant_a, scene_graph_a);
  auto checked_a = MultibodyPlantElements(plant_a, scene_graph_a);
  auto elem_b = MultibodyPlantElements::FromPlant(plant_b, scene_graph_b);
  auto checked_b = MultibodyPlantElements(plant_b, scene_graph_b);

  auto assert_body_equals = [&](const Body<double>& body_a,
                                const Body<double>& body_b,
                                bool check_inertia = true) {
    SCOPED_TRACE("check_body_equals");

    const auto *rigid_body_a = dynamic_cast<const RigidBody<double>*>(&body_a);
    const auto *rigid_body_b = dynamic_cast<const RigidBody<double>*>(&body_b);
    ASSERT_NE(rigid_body_a, nullptr);
    ASSERT_NE(rigid_body_b, nullptr);

    CheckElement(body_a, body_b);

    if (check_inertia) {
      EXPECT_EQ(rigid_body_a->default_spatial_inertia().CopyToFullMatrix6(),
          rigid_body_b->default_spatial_inertia().CopyToFullMatrix6());
    }

    EXPECT_THAT(checked_a.model_instances(),
                testing::Contains(body_a.model_instance()));
    EXPECT_THAT(checked_b.model_instances(),
                testing::Contains(body_b.model_instance()));
    EXPECT_EQ(plant_a->GetDefaultFreeBodyPose(body_a).GetAsMatrix4(),
              plant_b->GetDefaultFreeBodyPose(body_b).GetAsMatrix4());
    checked_a.bodies().insert(&body_a);
    checked_b.bodies().insert(&body_b);
  };

  auto assert_frame_equals = [&](const Frame<double>& frame_a,
                                 const Frame<double>& frame_b) {
    SCOPED_TRACE("assert_frame_equals");
    CheckElement(frame_a, frame_b, false);
    EXPECT_THAT(checked_a.bodies(), testing::Contains(&frame_a.body()));
    EXPECT_THAT(checked_b.bodies(), testing::Contains(&frame_b.body()));
    EXPECT_EQ(frame_a.GetFixedPoseInBodyFrame().GetAsMatrix4(),
              frame_b.GetFixedPoseInBodyFrame().GetAsMatrix4());
    checked_a.frames().insert(&frame_a);
    checked_b.frames().insert(&frame_b);
  };

  auto assert_joint_equals = [&](const Joint<double>& joint_a,
                                 const Joint<double>& joint_b) {
    SCOPED_TRACE("assert_joint_equals");
    EXPECT_THAT(checked_a.frames(),
                testing::Contains(&joint_a.frame_on_parent()));
    EXPECT_THAT(checked_b.frames(),
                testing::Contains(&joint_b.frame_on_parent()));
    EXPECT_THAT(checked_a.frames(),
                testing::Contains(&joint_a.frame_on_child()));
    EXPECT_THAT(checked_b.frames(),
                testing::Contains(&joint_b.frame_on_child()));
    EXPECT_EQ(joint_a.position_lower_limits(), joint_b.position_lower_limits());
    EXPECT_EQ(joint_a.position_upper_limits(), joint_b.position_upper_limits());
    EXPECT_EQ(joint_a.velocity_lower_limits(), joint_b.velocity_lower_limits());
    EXPECT_EQ(joint_a.velocity_upper_limits(), joint_b.velocity_upper_limits());
    EXPECT_EQ(joint_a.acceleration_lower_limits(),
              joint_b.acceleration_lower_limits());
    EXPECT_EQ(joint_a.acceleration_upper_limits(),
              joint_b.acceleration_upper_limits());
    EXPECT_EQ(joint_a.default_positions(), joint_b.default_positions());
    EXPECT_EQ(joint_a.default_positions(), joint_b.default_positions());

    int num_matches{};
    if (auto result =
            CastsTo<const BallRpyJoint<double>*, const PrismaticJoint<double>*,
                    const RevoluteJoint<double>*,
                    const UniversalJoint<double>*>(&joint_a)) {
      result.DoIfCastable([&](const auto* cast_joint_a) {
        auto cast_joint_b = dynamic_cast<decltype(cast_joint_a)>(&joint_b);
        ASSERT_NE(cast_joint_b, nullptr);
        EXPECT_EQ(cast_joint_a->damping(), cast_joint_b->damping());
        ++num_matches;
      });
    }
    if (auto result = CastsTo<const PrismaticJoint<double>*>(&joint_a)) {
      result.DoIfCastable([&](const auto* cast_joint_a) {
        auto cast_joint_b = dynamic_cast<decltype(cast_joint_a)>(&joint_b);
        ASSERT_NE(cast_joint_b, nullptr);
        EXPECT_EQ(cast_joint_a->translation_axis(),
                  cast_joint_b->translation_axis());
        ++num_matches;
      });
    }
    if (auto result = CastsTo<const RevoluteJoint<double>*>(&joint_a)) {
      result.DoIfCastable([&](const auto* cast_joint_a) {
        auto cast_joint_b = dynamic_cast<decltype(cast_joint_a)>(&joint_b);
        ASSERT_NE(cast_joint_b, nullptr);
        EXPECT_EQ(cast_joint_a->revolute_axis(), cast_joint_b->revolute_axis());
        ++num_matches;
      });
    }
    if (auto result = CastsTo<const WeldJoint<double>*>(&joint_a)) {
      result.DoIfCastable([&](const auto* cast_joint_a) {
        auto cast_joint_b = dynamic_cast<decltype(cast_joint_a)>(&joint_b);
        ASSERT_NE(cast_joint_b, nullptr);
        EXPECT_EQ(cast_joint_a->X_FM().GetAsMatrix4(),
                  cast_joint_b->X_FM().GetAsMatrix4());
        ++num_matches;
      });
    }
    EXPECT_GE(num_matches, 0);

    checked_a.joints().insert(&joint_a);
    checked_b.joints().insert(&joint_b);
  };

  auto assert_geometry_equals = [&](const GeometryId& a, const GeometryId& b) {
    const auto& inspector_a = scene_graph_a->model_inspector();
    const Body<double>* body_a =
        plant_a->GetBodyFromFrameId(inspector_a.GetFrameId(a));
    ASSERT_NE(body_a, nullptr);
    EXPECT_THAT(checked_a.bodies(), testing::Contains(body_a));
    auto geometry_a = inspector_a.CloneGeometryInstance(a);

    const auto& inspector_b = scene_graph_b->model_inspector();
    const Body<double>* body_b =
        plant_b->GetBodyFromFrameId(inspector_b.GetFrameId(b));
    ASSERT_NE(body_b, nullptr);
    EXPECT_THAT(checked_b.bodies(), testing::Contains(body_b));
    auto geometry_b = inspector_b.CloneGeometryInstance(b);

    EXPECT_EQ(geometry_a->name(), geometry_b->name());
    EXPECT_EQ(geometry_a->pose().GetAsMatrix4(),
              geometry_b->pose().GetAsMatrix4());

    AssertShapeEquals(geometry_a->shape(), geometry_b->shape());
    AssertPropertiesEquals(geometry_a->perception_properties(),
                           geometry_b->perception_properties());
    AssertPropertiesEquals(geometry_a->proximity_properties(),
                           geometry_b->proximity_properties());
    AssertPropertiesEquals(geometry_a->illustration_properties(),
                           geometry_b->illustration_properties());
  };

  auto assert_collision_filter_pair_equals =
      [&](const std::pair<GeometryId, GeometryId>& a,
          const std::pair<GeometryId, GeometryId>& b) {
        assert_geometry_equals(a.first, b.first);
        assert_geometry_equals(a.second, b.second);
      };

  auto frame_map = [](const SortedSet<const Frame<double>*>& frames) {
    std::map<std::pair<std::string, std::string>,
             std::set<const Frame<double>*>>
        out;
    for (const auto& frame : frames) {
      auto key = std::make_pair(frame->body().name(), frame->name());
      out[key].insert(frame);
    }
    return out;
  };

  for (const auto [a, b] :
       Zip(elem_a.model_instances(), elem_b.model_instances())) {
    ASSERT_NE(&a, &b);
    ASSERT_EQ(a, b);
    checked_a.model_instances().insert(a);
    checked_b.model_instances().insert(b);
  }

  for (const auto [a, b] : Zip(elem_a.bodies(), elem_b.bodies())) {
    if (a == &plant_a->world_body() || b == &plant_b->world_body()) {
      EXPECT_EQ(a, &plant_a->world_body());
      EXPECT_EQ(b, &plant_b->world_body());
      assert_body_equals(*a, *b, false);
    } else {
      assert_body_equals(*a, *b);
    }
  }
  // N.B. Because frame indices can be shifted when adding bodies, we cannot
  // trust this ordering. Instead, we need to find an identifier.
  auto frame_map_a = frame_map(elem_a.frames());
  auto frame_map_b = frame_map(elem_b.frames());
  ASSERT_EQ(frame_map_a.size(), frame_map_b.size());

  for (const auto& [key, frames_a] : frame_map_a) {
    auto frames_b = frame_map_b[key];
    for (const auto [frame_a, frame_b] : Zip(frames_a, frames_b)) {
      assert_frame_equals(*frame_a, *frame_b);
    }
  }

  for (const auto [joint_a, joint_b] : Zip(elem_a.joints(), elem_b.joints())) {
    assert_joint_equals(*joint_a, *joint_b);
  }
  for (const auto [joint_actuator_a, joint_actuator_b] :
       Zip(elem_a.joint_actuators(), elem_b.joint_actuators())) {
    EXPECT_EQ(joint_actuator_a->effort_limit(),
              joint_actuator_b->effort_limit());
  }

  if (scene_graph_b != nullptr) {
    for (const auto [geometry_id_a, geometry_id_b] :
         Zip(elem_a.geometry_ids(), elem_b.geometry_ids())) {
      assert_geometry_equals(geometry_id_a, geometry_id_b);
    }

    for (const auto [collision_filter_pair_a, collision_filter_pair_b] :
         Zip(elem_a.collision_filter_pairs(),
             elem_b.collision_filter_pairs())) {
      assert_collision_filter_pair_equals(collision_filter_pair_a,
                                          collision_filter_pair_b);
    }
  }
}

// Compares the poses of two frames
template<typename ...Ts>
void CompareFramePoses(const MultibodyPlant<double>& plant,
                       const systems::Context<double>& context,
                       const MultibodyPlant<double>& sub_plant,
                       const systems::Context<double>& sub_context,
                       const std::string_view& base_frame_name,
                       const std::string_view& test_frame_name,
                       Ts...args) {
  auto X_BT_sub = sub_plant.CalcRelativeTransform(
      sub_context, sub_plant.GetFrameByName(base_frame_name),
      sub_plant.GetFrameByName(test_frame_name));
  auto X_BT = plant.CalcRelativeTransform(
      context, plant.GetFrameByName(base_frame_name, args...),
      plant.GetFrameByName(test_frame_name, args...));

  EXPECT_TRUE(
      CompareMatrices(X_BT_sub.GetAsMatrix4(), X_BT.GetAsMatrix4(), 1e-10));
}

constexpr std::array kJointTypeNameList{
    PrismaticJoint<double>::kTypeName,
    RevoluteJoint<double>::kTypeName,
    BallRpyJoint<double>::kTypeName,
    UniversalJoint<double>::kTypeName,
    WeldJoint<double>::kTypeName,
};

class ArbitraryMultibodyStuffBuilder {
 public:
  explicit ArbitraryMultibodyStuffBuilder(MultibodyPlant<double>* plant)
      : plant_(plant) {}

  void Build(int num_bodies = 30, bool slight_difference = false) {
    // Add ground plane.
    auto X_FH = HalfSpace::MakePose({0, 0, 1}, {0, 0, 0});
    plant_->RegisterCollisionGeometry(plant_->world_body(), X_FH, HalfSpace(),
                                      "ground_plane_collision",
                                      CoulombFriction(0.8, 0.3));

    int grid_rows = 5;
    const Body<double>* prev_body = nullptr;
    for (int i = 0; i < num_bodies; ++i) {
      rg_.seed(i);
      const RigidBody<double>* body = &RandomBody();
      int grid_col = i % grid_rows;
      int grid_row = i / grid_rows;
      if (slight_difference) {
        grid_row += 1;
      }
      plant_->SetDefaultFreeBodyPose(
          *body, RigidTransformd(Eigen::Vector3d(grid_col, grid_row, 2)));
      RandomFrame(body->body_frame());
      // Consider attaching a joint and/or frame to the world.
      if (Maybe() || num_bodies < 3) {
        prev_body = &plant_->world_body();
        RandomFrame(plant_->world_frame());
      }
      if (prev_body != nullptr && (Maybe() || num_bodies < 3)) {
        const Joint<double>& joint = RandomJoint(*prev_body, *body);
        if (joint.num_velocities() == 1 && (Maybe() || num_bodies < 3)) {
          RandomJointActuator(joint);
        }
      }
      if (plant_->geometry_source_is_registered()) {
        RandomGeometry(*body);

        if ((Maybe() || num_bodies < 3) && prev_body != nullptr &&
            prev_body != &plant_->world_body()) {
          geometry::GeometrySet set_a(
              plant_->GetCollisionGeometriesForBody(*prev_body));
          geometry::GeometrySet set_b(
              plant_->GetCollisionGeometriesForBody(*body));

          plant_->ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
              {"a", set_a}, {"b", set_b});
        }
      }
      prev_body = body;
    }
  }

 private:
  // Increments for a given key.
  int NextIndex(const std::string& key = "") { return ++count_[key]; }

  double Random(double a = 0., double b = 1.) {
    return std::uniform_real_distribution<>(a, b)(rg_);
  }

  // Return true or false randomly
  bool Maybe() { return std::uniform_int_distribution<>(0, 1)(rg_) == 1; }

  // Returns a "random" model instance (by incrementing).
  ModelInstanceIndex RandomModelInstance() {
    return plant_->AddModelInstance(fmt::format("model_{}", NextIndex()));
  }

  // Returns a "random" position.
  Eigen::Vector3d RandomPosition() { return {0.2 * Random(), 0, 0}; }

  // Returns a "random" pose.
  RigidTransformd RandomX() { return RigidTransformd(RandomPosition()); }
  // Returns a "random" body, with an incrementing name.
  const RigidBody<double>& RandomBody() {
    SpatialInertia<double> inertia(
        Random(0.2, 1.), RandomPosition(),
        UnitInertia<double>(Random(0.2, 0.3), Random(0.2, 0.3),
                            Random(0.2, 0.3)));
    return plant_->AddRigidBody(fmt::format("body_{}", NextIndex()),
                                RandomModelInstance(), inertia);
  }

  // Returns a "random" frame, with an incrementing name.
  const Frame<double>& RandomFrame(const Frame<double>& parent_frame) {
    return plant_->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
        fmt::format("frame_{}", NextIndex()), parent_frame, RandomX(),
        parent_frame.model_instance()));
  }

  // Returns a "random" joint, but with an incrementing name. Note that we
  // use a separate index so that we ensure we can loop through all
  // joints.
  const Joint<double>& RandomJoint(const Body<double>& parent,
                                   const Body<double>& child) {
    auto index = NextIndex("joint");
    const auto name = fmt::format("joint_{}", index);
    const auto joint_type =
        kJointTypeNameList.at(index % kJointTypeNameList.size());
    const Frame<double>& frame_on_parent = RandomFrame(parent.body_frame());
    const Frame<double>& frame_on_child = RandomFrame(child.body_frame());
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    axis[NextIndex() % 3] = 1;
    const double damping = Random();
    std::unique_ptr<Joint<double>> joint;
    if (joint_type == BallRpyJoint<double>::kTypeName) {
      joint = std::make_unique<BallRpyJoint<double>>(name, frame_on_parent,
                                                     frame_on_child, damping);
    } else if (joint_type == PrismaticJoint<double>::kTypeName) {
      joint = std::make_unique<PrismaticJoint<double>>(
          name, frame_on_parent, frame_on_child, axis, damping);
    } else if (joint_type == RevoluteJoint<double>::kTypeName) {
      joint = std::make_unique<RevoluteJoint<double>>(
          name, frame_on_parent, frame_on_child, axis, damping);
    } else if (joint_type == UniversalJoint<double>::kTypeName) {
      joint = std::make_unique<UniversalJoint<double>>(name, frame_on_parent,
                                                       frame_on_child, damping);
    } else if (joint_type == WeldJoint<double>::kTypeName) {
      joint = std::make_unique<WeldJoint<double>>(name, frame_on_parent,
                                                  frame_on_child, RandomX());
    } else {
      DRAKE_DEMAND(false);
    }
    return plant_->AddJoint(std::move(joint));
  }

  // Creates a "random" joint actuator.
  const JointActuator<double>& RandomJointActuator(const Joint<double>& joint) {
    return plant_->AddJointActuator(fmt::format("actuator_{}", NextIndex()),
                                    joint, Random(1, 2));
  }
  // Creates a random geometry.
  void RandomGeometry(const Body<double>& body) {
    auto i = NextIndex();
    geometry::Box box(Random(0.1, 0.3), Random(0.1, 0.3), Random(0.1, 0.3));
    plant_->RegisterVisualGeometry(body, RandomX(), box,
                                   fmt::format("visual_{}", i),
                                   {Random(), 0, 0, 0.75});
    double static_friction = Random(0.1, 1.);
    plant_->RegisterCollisionGeometry(
        body, RandomX(), box, fmt::format("collision_{}", i),
        CoulombFriction(static_friction, static_friction / 2));
  }

  std::unordered_map<std::string, int> count_;
  std::mt19937 rg_{0};

  MultibodyPlant<double>* plant_;
};

// Deterministic, physically valid, jumble of arbitrary stuff.
//
// The goal of this factory is to:
//
// - Produce a set of elements and cases that exercise each code path in
//   `MultibodyPlantSubgraph.add_to` and `MultibodyPlantElementsMap`.
// - Ensure each element has somewhat "random" but unique properties for each
//   element produced.
// - Allow for a slight difference to show that strict plant comparison can be
//   falsified.
void AddArbitraryMultibodyStuff(MultibodyPlant<double>* plant,
                                int num_bodies = 30,
                                bool slight_difference = false) {
  ArbitraryMultibodyStuffBuilder(plant).Build(num_bodies, slight_difference);
}

class APITest : public ::testing::Test {
 public:
  static std::tuple<MultibodyPlant<double>*, SceneGraph<double>*,
                    std::variant<std::unique_ptr<systems::Diagram<double>>,
                                 std::unique_ptr<DiagramBuilder<double>>>>
  MakeArbitraryMultibodyStuff(bool finalize = true, int num_bodies = 30,
                              bool slight_difference = false) {
    auto builder = std::make_unique<DiagramBuilder<double>>();
    auto result = AddMultibodyPlantSceneGraph(builder.get(), 0.01);
    AddArbitraryMultibodyStuff(&result.plant, num_bodies, slight_difference);
    if (finalize) {
      result.plant.Finalize();
      auto diagram = builder->Build();
      return {&result.plant, &result.scene_graph, std::move(diagram)};
    }
    return {&result.plant, &result.scene_graph, std::move(builder)};
  }
};


class SilentTestRunner : public testing::EmptyTestEventListener {
  void OnTestPartResult(
      const testing::TestPartResult& test_part_result) override {
    if (test_part_result.failed()) {
      has_failed = true;
      throw testing::AssertionException(test_part_result);
    }
  }

 public:
  bool has_failed{false};
};

testing::AssertionResult CheckForAnyFailure(
    const ::testing::TestPartResultArray& results) {
  int num_failures = 0;
  for (int gi = 0; gi < results.size(); ++gi) {
    if (results.GetTestPartResult(gi).failed()) {
      ++num_failures;
    }
  }
  if (num_failures == 0) {
    return testing::AssertionFailure() << "No failures found";
  } else {
    return testing::AssertionSuccess()
           << "Found " << num_failures << " failures";
  }
}
// This machinery is needed to run a negative test, i.e., to be able
// to call a function containg EXPECT_* and ASSERT_* calls with the expectation
// that some of those will be failures.
//
// Using this machinery allows us to capture the results from running
// a test without being reported to the overall test result aggregator. We can
// then inspect the results to see if there were any failures.
#define EXPECT_TO_FAIL(statement) \
  do {\
    ::testing::TestPartResultArray gtest_results;\
    {\
      ::testing::ScopedFakeTestPartResultReporter gtest_reporter(\
          ::testing::ScopedFakeTestPartResultReporter:: \
          INTERCEPT_ONLY_CURRENT_THREAD, &gtest_results);\
      { statement; }\
    }\
    EXPECT_TRUE(CheckForAnyFailure(gtest_results)); \
  } while (::testing::internal::AlwaysFalse())

// Ensures that the test model is logical and can be compared / falsified
TEST_F(APITest, MetaArbitraryMultibodyStuff) {
  auto [plant_a, scene_graph_a, builder_a] = MakeArbitraryMultibodyStuff(false);

  // Ensure that we span all relevant joint classes.
  std::set<std::string> joint_type_set;
  for (const auto* joint : GetJoints(*plant_a)) {
    joint_type_set.insert(joint->type_name());
  }
  EXPECT_EQ(joint_type_set, std::set<std::string>(kJointTypeNameList.begin(),
                                                  kJointTypeNameList.end()));

  {
    // Ensure that we can copy via a subgraph pre-Finalize.
    DiagramBuilder<double> builder_b;
    auto [plant_b, scene_graph_b] =
        AddMultibodyPlantSceneGraph(&builder_b, plant_a->time_step());

    MultibodyPlantSubgraph subgraph_a(
        MultibodyPlantElements::FromPlant(plant_a, scene_graph_a));
    subgraph_a.AddTo(&plant_b);
    // Check equivalence.
    SCOPED_TRACE("Check Equivalence");
    AssertPlantEquals(plant_a, scene_graph_a, &plant_b, &scene_graph_b);
  }
  // Ensure that this is "physically" valid.
  plant_a->Finalize();

  // Checking for determinism, making a slight change to trigger an error.
  for (bool slight_difference : {false, true}) {
    auto [plant_b, scene_graph_b, builder_b] =
        MakeArbitraryMultibodyStuff(true, 30, slight_difference);

    if (!slight_difference) {
      SCOPED_TRACE("No slight_difference");
      AssertPlantEquals(plant_a, scene_graph_a, plant_b, scene_graph_b);
    } else {
      SCOPED_TRACE("With slight_difference");
      EXPECT_TO_FAIL(
          AssertPlantEquals(plant_a, scene_graph_a, plant_b, scene_graph_b));
    }
  }
}

TEST_F(APITest, MultibodyPlantElements) {
  auto [plant, scene_graph, builder] = MakeArbitraryMultibodyStuff(true, 1);

  // Test nominal usage.
  auto elem = MultibodyPlantElements::FromPlant(plant, scene_graph);
  MultibodyPlantElementsCount actual_count(elem);
  MultibodyPlantElementsCount expected_count(3, 2, 6, 1, 1, 3);
  EXPECT_EQ(expected_count, actual_count);

  // Test copying.
  MultibodyPlantElements elem_copy(elem);
  EXPECT_EQ(elem, elem_copy);

  // Test subgraph invariant.
  EXPECT_NO_THROW(CheckSubgraphInvariants(elem));

  // Helper struct that runs a negative test on CheckSubgraphInvariants when it
  // goes out of scope.
  struct ScopedCheckSubgraphNegativeTest {
    explicit ScopedCheckSubgraphNegativeTest(MultibodyPlantElements elem)
        : elem_copy(std::move(elem)) {}

    ~ScopedCheckSubgraphNegativeTest() {
      EXPECT_TO_FAIL(EXPECT_NO_THROW(CheckSubgraphInvariants(elem_copy)));
    }
    MultibodyPlantElements elem_copy;
  };

  // Check negative cases:
  // - subgraph model instance in plant model instances
  {
    SCOPED_TRACE("Add ModelInstance(100)");
    ScopedCheckSubgraphNegativeTest test(elem);
    test.elem_copy.model_instances().insert(ModelInstanceIndex(100));
  }
  // - subgraph bodies in subgraph model instances.
  {
    SCOPED_TRACE("Remove world_model_instance");
    ScopedCheckSubgraphNegativeTest test(elem);
    test.elem_copy.model_instances().erase(world_model_instance());
  }
  // - subgraph element must be part of the subgraph plant.
  {
    // TODO(azeey) The python prototype has a test equivalent to this, but
    // because std::set does not allow duplicates, the test fails here.

    // MultibodyPlant<double>* other_plant{nullptr};
    // auto [other_result, other_builder] = MakeArbitraryMultibodyStuff(true,
    // 1);
    // other_plant = &other_result.plant;
    // SCOPED_TRACE("Add world_body");
    // ScopedCheckSubgraphNegativeTest test(elem);
    // test.elem_copy.bodies().insert(&other_plant->world_body());
  }

  // - subgraph frames must be attached to subgraph bodies
  // - subgraph joints only connected to subgraph bodies
  // - subgrpah geometries must be attached to subgraph bodies
  {
    SCOPED_TRACE("Remove world_body");
    ScopedCheckSubgraphNegativeTest test(elem);
    test.elem_copy.bodies().erase(&plant->world_body());
  }
  // - subgraph joint actuators must solely act on subgraph joints
  {
    SCOPED_TRACE("Remove first joint");
    ScopedCheckSubgraphNegativeTest test(elem);
    auto joint = *elem.joints().begin();
    test.elem_copy.joints().erase(joint);
  }
  // Test usage without SceneGraph.
  MultibodyPlantElements elem_copy_no_scene_graph(elem);
  elem_copy_no_scene_graph.ResetSceneGraph();
  const auto from_plant = MultibodyPlantElements::FromPlant(plant);
  EXPECT_EQ(from_plant, elem_copy_no_scene_graph);

  EXPECT_ANY_THROW(elem += elem_copy);

  // Tests that we can add "disjoint" stuff.
  elem_copy = elem;
  auto last_body = *elem.bodies().rbegin();
  elem_copy.bodies().erase(last_body);
  EXPECT_NE(elem, elem_copy);
  MultibodyPlantElements elem_world_body_only(plant, scene_graph);
  elem_world_body_only.bodies().insert(last_body);
  elem_copy += elem_world_body_only;
  EXPECT_EQ(elem, elem_copy);
}

MultibodyPlantElementsMap CreateManualMap(MultibodyPlant<double>* plant_a,
                                          SceneGraph<double>* scene_graph_a,
                                          MultibodyPlant<double>* plant_b,
                                          SceneGraph<double>* scene_graph_b) {
  // Manually construct map.
  MultibodyPlantElementsMap a_to_b(plant_a, plant_b, scene_graph_a);

  MultibodyPlantElements empty_a(plant_a, scene_graph_a);
  EXPECT_EQ(empty_a, a_to_b.MakeEmptyElementsSrc());

  MultibodyPlantElements empty_b(plant_b, scene_graph_b);
  EXPECT_EQ(empty_b, a_to_b.MakeEmptyElementsSrc());

  MultibodyPlantElementsMap b_to_a(plant_b, plant_a, scene_graph_b);
  auto elem_a = MultibodyPlantElements::FromPlant(plant_a, scene_graph_a);
  auto elem_b = MultibodyPlantElements::FromPlant(plant_b, scene_graph_b);

  auto populate_maps = [&](const auto& field) {
    for (const auto& [a, b] : Zip(field(elem_a), field(elem_b))) {
      field(a_to_b)[a] = b;
      field(b_to_a)[b] = a;
    }
  };
  populate_maps([](auto& obj) -> auto& { return obj.model_instances(); });
  populate_maps([](auto& obj) -> auto& { return obj.bodies(); });
  populate_maps([](auto& obj) -> auto& { return obj.frames(); });
  populate_maps([](auto& obj) -> auto& { return obj.joints(); });
  populate_maps([](auto& obj) -> auto& { return obj.joint_actuators(); });
  populate_maps([](auto& obj) -> auto& { return obj.geometry_ids(); });

  EXPECT_EQ(MultibodyPlantElementsCount(a_to_b),
            MultibodyPlantElementsCount(elem_a));
  EXPECT_NE(a_to_b, b_to_a);
  // TODO(azeey) Uncomment the following when
  // MultibodyPlantElementsMap::Inverse is implemented.
  // EXPECT_EQ(a_to_b, b_to_a.Inverse())
  // EXPECT_EQ(b_to_a, a_to_b.Inverse())
  return a_to_b;
}

// Tests basic container API for MultibodyPlantElementsMap.
// All `Copy*` functionality is tested (implicitly) in
// `SubgraphAddToCopying`.
TEST_F(APITest, MultibodyPlantElementsMap) {
  auto [plant_a, scene_graph_a, builder_a] =
      MakeArbitraryMultibodyStuff(true, 1);
  auto [plant_b, scene_graph_b, builder_b] =
      MakeArbitraryMultibodyStuff(true, 1);

  ASSERT_NE(plant_a, plant_b);
  ASSERT_NE(scene_graph_a, scene_graph_b);
  AssertPlantEquals(plant_a, scene_graph_a, plant_b, scene_graph_b);
  CreateManualMap(plant_a, scene_graph_a, plant_b, scene_graph_b);
}

// Ensures that we always have a "subgraph" on construction and with mutation.
TEST_F(APITest, SubgraphConstructionAndMutation) {
  auto [plant, scene_graph, builder] = MakeArbitraryMultibodyStuff(true, 10);

  MultibodyPlantSubgraph subgraph(
      MultibodyPlantElements::FromPlant(plant, scene_graph));

  const auto& elem = subgraph.elements_src();
  // elem contains:
  //   model_instances: 12,
  //   bodies: 11,
  //   frames: 32,
  //   joints: 4,
  //   joint_actuators: 1,
  //   geometry_ids: 21
  // Note, this is different from the python implementation due to the
  // difference in random number generation.
  EXPECT_NO_THROW(CheckSubgraphInvariants(elem));

  // RemoveBody
  {
    MultibodyPlantSubgraph tmp(subgraph);
    auto* body = &plant->world_body();
    MultibodyPlantElements elem_removed = tmp.RemoveBody(body);
    EXPECT_NO_THROW(CheckSubgraphInvariants(tmp.elements_src()));
    EXPECT_THAT(elem_removed.bodies(), testing::Contains(body));
    EXPECT_THAT(tmp.elements_src().bodies(),
                testing::Not(testing::Contains(body)));
    MultibodyPlantElementsCount expected_count(0, 1, 7, 2, 0, 1);
    EXPECT_EQ(expected_count, MultibodyPlantElementsCount(elem_removed));
  }

  // RemoveFrame
  {
    MultibodyPlantSubgraph tmp(subgraph);
    const auto* frame = &plant->world_frame();
    MultibodyPlantElements elem_removed = tmp.RemoveFrame(frame);
    CheckSubgraphInvariants(tmp.elements_src());
    EXPECT_THAT(elem_removed.frames(), testing::Contains(frame));
    EXPECT_THAT(tmp.elements_src().frames(),
                testing::Not(testing::Contains(frame)));
    MultibodyPlantElementsCount expected_count(0, 0, 1, 0, 0, 0);
    EXPECT_EQ(expected_count, MultibodyPlantElementsCount(elem_removed));
  }
  // RemoveJoint
  {
    MultibodyPlantSubgraph tmp(subgraph);
    const auto* joint = *elem.joints().begin();
    MultibodyPlantElements elem_removed = tmp.RemoveJoint(joint);
    CheckSubgraphInvariants(tmp.elements_src());
    EXPECT_THAT(elem_removed.joints(), testing::Contains(joint));
    EXPECT_THAT(tmp.elements_src().joints(),
                testing::Not(testing::Contains(joint)));
    MultibodyPlantElementsCount expected_count(0, 0, 0, 1, 1, 0);
    EXPECT_EQ(expected_count, MultibodyPlantElementsCount(elem_removed));
  }
  // TODO(eric.cousineau): Test remove_joint_actuator, remove_geometry_id
}

// Connects a zero-torque input to a given model instance in a plant.
void BuildWithNoControl(DiagramBuilder<double>* builder,
                        MultibodyPlant<double>* plant,
                        ModelInstanceIndex model) {
  // TODO(eric.cousineau): Use `multibody_plant_prototypes.control` if the
  // dependency can be simplified.
  auto nu = plant->num_actuated_dofs(model);
  auto *constant = builder->AddSystem<systems::ConstantVectorSource<double>>(
      Eigen::VectorXd::Zero(nu));
  builder->Connect(constant->get_output_port(),
                   plant->get_actuation_input_port(model));
}

// Ensures that index ordering is generally the same when copying a plant using
// a MultibodyPlantSubgraph.AddTo.
TEST_F(APITest, SubgraphAddToCopying) {
  // TODO(eric.cousineau): Increase number of bodies for viz, once
  // `create_manual_map` can acommodate it.
  auto [plant_a, scene_graph_a, builder_a] =
      MakeArbitraryMultibodyStuff(true, 1);
  // Check for general ordering with full subgraph "cloning".
  DiagramBuilder<double> builder_b;
  auto [plant_b, scene_graph_b] = AddMultibodyPlantSceneGraph(
      &builder_b, plant_a->time_step());
  MultibodyPlantSubgraph subgraph_a(MultibodyPlantElements::FromPlant(
      plant_a, scene_graph_a));
  auto a_to_b = subgraph_a.AddTo(&plant_b);
  plant_b.Finalize();
  AssertPlantEquals(plant_a, scene_graph_a, &plant_b, &scene_graph_b);

  auto a_to_b_expected =
      CreateManualMap(plant_a, scene_graph_a, &plant_b, &scene_graph_b);
  EXPECT_EQ(a_to_b, a_to_b_expected);

  if (FLAGS_visualize) {
    for (const auto& model : GetModelInstances(plant_b)) {
      BuildWithNoControl(&builder_b, &plant_b, model);
    }

    drake::log()->trace("SubgraphAddToCopying");
    geometry::DrakeVisualizer<double>::AddToBuilder(&builder_b, scene_graph_b);
    auto diagram = builder_b.Build();
    systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(1.0);
    simulator.Initialize();
    diagram->Publish(simulator.get_context());
    simulator.AdvanceTo(1.0);
  }
}

TEST_F(APITest, FrameRename) {
  MultibodyPlant<double> plant(0.01);
  const std::string sdf_string = R"""(
  <sdf version='1.7'>
    <model name='frame_rename_test'>
      <frame name='my_frame'/>
      <link name='my_link'/>
    </model>
  </sdf>)""";

  Parser(&plant).AddModelFromString(sdf_string, "sdf");
  EXPECT_TRUE(plant.HasFrameNamed("my_frame"));

  MultibodyPlantSubgraph subgraph(MultibodyPlantElements::FromPlant(&plant));
  auto frame_name_remap = [](const Frame<double>& frame) {
    return frame.name() + "_renamed";
  };

  MultibodyPlant<double> plant_dest(0.01);
  subgraph.AddTo(&plant_dest, std::nullopt, frame_name_remap);
  EXPECT_FALSE(plant_dest.HasFrameNamed("my_frame"));
  EXPECT_TRUE(plant_dest.HasFrameNamed("my_frame_renamed"));
}

TEST_F(APITest, ModelInstanceRemap) {
  MultibodyPlant<double> plant(0.01);
  const std::string sdf_string = R"""(
  <sdf version='1.7'>
    <model name='frame_rename_test'>
      <frame name='my_frame'/>
      <link name='link1'/>
      <link name='link2'/>
      <joint name='j1' type='fixed'>
        <parent>link1</parent>
        <child>link2</child>
      </joint>
    </model>
  </sdf>)""";

  Parser(&plant).AddModelFromString(sdf_string, "sdf");

  MultibodyPlant<double> plant_dest(0.01);
  plant_dest.AddModelInstance("model_1");
  plant_dest.AddModelInstance("model_2");
  ModelInstanceIndex dest_model =
      plant_dest.AddModelInstance("destination_model");
  EXPECT_EQ(dest_model, ModelInstanceIndex(4));
  auto model_remap = [&] (ModelInstanceIndex model_instance){
    return dest_model;
  };

  MultibodyPlantSubgraph subgraph(MultibodyPlantElements::FromPlant(&plant));

  subgraph.AddTo(&plant_dest, model_remap);
  EXPECT_TRUE(plant_dest.HasFrameNamed("my_frame", dest_model));
  EXPECT_TRUE(plant_dest.HasBodyNamed("link1", dest_model));
  EXPECT_TRUE(plant_dest.HasBodyNamed("link2", dest_model));
  EXPECT_TRUE(plant_dest.HasJointNamed("j1", dest_model));
}

class TestWorkflows: public APITest {};

// Tests subgraphs (post-finalize) without a scene graph.
TEST_F(TestWorkflows, CompositionArrayWithoutSceneGraph) {
  MultibodyPlant<double> iiwa_plant(0.01);
  const std::string iiwa_file = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_spheres_dense_elbow_collision.urdf");
  auto iiwa_model = Parser(&iiwa_plant).AddModelFromFile(iiwa_file, "iiwa");
  iiwa_plant.Finalize();
  auto iiwa_context = iiwa_plant.CreateDefaultContext();
  const auto &base_frame_sub = iiwa_plant.GetFrameByName("base");

  // N.B. Because the model is not welded, we do not an additional policy
  // to "disconnect" it.
  MultibodyPlantSubgraph iiwa_subgraph(
      MultibodyPlantElements::FromPlant(&iiwa_plant));

  // Make 10 copies of the IIWA in a line.
  MultibodyPlant<double> plant(0.01);
  std::vector<ModelInstanceIndex> models;
  for (int i = 0; i < 10; ++i) {
    auto sub_to_full = iiwa_subgraph.AddTo(&plant, fmt::format("iiwa_{}", i));
    RigidTransformd X_WB(Eigen::Vector3d(i * 0.5, 0, 0));
    const auto *base_frame = sub_to_full.frames()[&base_frame_sub];
    plant.WeldFrames(plant.world_frame(), *base_frame, X_WB);
    auto model = sub_to_full.model_instances()[iiwa_model];
    models.push_back(model);
  }
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  for (std::size_t i = 0; i < models.size(); ++i) {
    ModelInstanceIndex model = models[i];
    EXPECT_EQ(plant.GetModelInstanceName(model), fmt::format("iiwa_{}", i));
    CompareFramePoses(plant, *context, iiwa_plant, *iiwa_context, "base",
                      "iiwa_link_7", model);
  }
}

// Tests subgraphs (post-finalize) with a scene graph. This time, using sugar
// from parse_as_multibody_plant_subgraph.
TEST_F(TestWorkflows, CompositionArrayWithSceneGraph) {
  // Create IIWA.
  const std::string iiwa_file = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_spheres_dense_elbow_collision.urdf");
  DiagramBuilder<double> iiwa_builder;

  auto [iiwa_plant, iiwa_scene_graph] =
      AddMultibodyPlantSceneGraph(&iiwa_builder, 0.1);
  const auto iiwa_model =
      Parser(&iiwa_plant).AddModelFromFile(iiwa_file, "iiwa");
  MultibodyPlantSubgraph iiwa_subgraph(
      MultibodyPlantElements::FromPlant(&iiwa_plant, &iiwa_scene_graph));
  // Make 10 copies of the IIWA in a line.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder, iiwa_plant.time_step());
  std::vector<ModelInstanceIndex> models;
  for (int i = 0; i < 10; ++i) {
    auto sub_to_full = iiwa_subgraph.AddTo(&plant, fmt::format("iiwa_{}", i));
    RigidTransformd X_WB(Eigen::Vector3d(i * 0.5, 0, 0));
    auto model = sub_to_full.model_instances()[iiwa_model];
    const auto& base_frame = plant.GetFrameByName("base", model);
    plant.WeldFrames(plant.world_frame(), base_frame, X_WB);
    models.push_back(model);
  }

  plant.Finalize();
  if (FLAGS_visualize) {
    drake::log()->trace("CompositionArrayWithSceneGraph");
    geometry::DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);
  }

  auto diagram = builder.Build();
  auto d_context = diagram->CreateDefaultContext();

  for (std::size_t i = 0; i < models.size(); ++i) {
    ModelInstanceIndex model = models[i];
    EXPECT_EQ(plant.GetModelInstanceName(model), fmt::format("iiwa_{}", i));
  }

  if (FLAGS_visualize) {
    std::cout << "  Visualize composite" << std::endl;
    Simulator<double> simulator(*diagram, d_context->Clone());
    simulator.Initialize();
    std::cout << "    Press enter...";
    std::cin.ignore();
  }
}

// Tests subgraphs (pre-finalize) for composition, with a scene graph, welding
// bodies together across different subgraphs.
TEST_F(TestWorkflows, CompositionGripperWorkflow) {
  // N.B. The frame-welding is done so that we can easily set the
  // positions of the IIWA / WSG without having to worry about / work
  // around the floating body coordinates.

  // Create IIWA.
  DiagramBuilder<double> iiwa_builder;
  auto [iiwa_plant, iiwa_scene_graph] =
      AddMultibodyPlantSceneGraph(&iiwa_builder, 0.);
  const std::string iiwa_file = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_spheres_dense_elbow_collision.urdf");
  const auto iiwa_model =
      Parser(&iiwa_plant).AddModelFromFile(iiwa_file, "iiwa");
  iiwa_plant.WeldFrames(iiwa_plant.world_frame(),
                        iiwa_plant.GetFrameByName("base"));
  iiwa_plant.Finalize();
  if (FLAGS_visualize) {
    std::cout << "test_composition" << std::endl;
    DrakeVisualizerd::AddToBuilder(&iiwa_builder, iiwa_scene_graph);
  }
  auto iiwa_diagram = iiwa_builder.Build();

  MultibodyPlantSubgraph iiwa_subgraph(
      MultibodyPlantElements::FromPlant(&iiwa_plant, &iiwa_scene_graph));
  iiwa_subgraph.RemoveBody(&iiwa_plant.world_body());

  // Create WSG.
  DiagramBuilder<double> wsg_builder;
  auto [wsg_plant, wsg_scene_graph] =
      AddMultibodyPlantSceneGraph(&wsg_builder, 0.);
  const std::string wsg_file = FindResourceOrThrow(
      "drake/manipulation/models/wsg_50_description/sdf/"
      "schunk_wsg_50.sdf");
  const auto wsg_model =
      Parser(&wsg_plant).AddModelFromFile(wsg_file, "gripper_model");
  wsg_plant.WeldFrames(wsg_plant.world_frame(),
                       wsg_plant.GetFrameByName("__model__"));
  wsg_plant.Finalize();
  if (FLAGS_visualize) {
    DrakeVisualizerd::AddToBuilder(&wsg_builder, wsg_scene_graph);
  }
  auto wsg_diagram = wsg_builder.Build();

  MultibodyPlantSubgraph wsg_subgraph(
      MultibodyPlantElements::FromPlant(&wsg_plant, &wsg_scene_graph));
  wsg_subgraph.RemoveBody(&wsg_plant.world_body());

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 1e-3);

  auto iiwa_to_plant = iiwa_subgraph.AddTo(&plant);
  auto wsg_to_plant = wsg_subgraph.AddTo(&plant);

  if (FLAGS_visualize) {
    DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
  }

  Eigen::Vector3d rpy_deg(90., 0., 90);
  math::RigidTransformd X_7G(math::RollPitchYawd(rpy_deg * M_PI / 180),
                             Eigen::Vector3d(0, 0, 0.114));
  const auto& frame_7 = plant.GetFrameByName("iiwa_link_7");
  const auto& frame_G = plant.GetFrameByName("body");
  plant.WeldFrames(frame_7, frame_G, X_7G);
  plant.Finalize();

  Eigen::VectorXd q_iiwa(7);
  q_iiwa << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  Eigen::Vector2d q_wsg(-0.03, 0.03);

  std::unique_ptr<Context<double>> iiwa_d_context =
      iiwa_diagram->CreateDefaultContext();
  Context<double>& iiwa_context =
      iiwa_plant.GetMyMutableContextFromRoot(iiwa_d_context.get());
  iiwa_plant.SetPositions(&iiwa_context, iiwa_model, q_iiwa);

  std::unique_ptr<Context<double>> wsg_d_context =
      wsg_diagram->CreateDefaultContext();
  Context<double>& wsg_context =
      wsg_plant.GetMyMutableContextFromRoot(wsg_d_context.get());
  wsg_plant.SetPositions(&wsg_context, wsg_model, q_wsg);

  // Transfer state and briefly compare.
  auto diagram = builder.Build();
  std::unique_ptr<Context<double>> d_context = diagram->CreateDefaultContext();
  Context<double>& context = plant.GetMyMutableContextFromRoot(d_context.get());

  iiwa_to_plant.CopyState(iiwa_context, &context);
  wsg_to_plant.CopyState(wsg_context, &context);

  // Compare frames from sub-plants.
  CompareFramePoses(plant, context, iiwa_plant, iiwa_context, "base",
                    "iiwa_link_7");
  CompareFramePoses(plant, context, wsg_plant, wsg_context, "body",
                    "left_finger");

  // Visualize.
  if (FLAGS_visualize) {
    std::cout << "  Visualize IIWA" << std::endl;
    Simulator<double>(*iiwa_diagram, iiwa_d_context->Clone()).Initialize();
    std::cout << "    Press enter...";
    std::cin.ignore();
    std::cout << "  Visualize WSG" << std::endl;
    Simulator<double>(*wsg_diagram, wsg_d_context->Clone()).Initialize();
    std::cout << "    Press enter...";
    std::cin.ignore();
    std::cout << "  Visualize Composite" << std::endl;
    Simulator<double>(*diagram, d_context->Clone()).Initialize();
    std::cout << "    Press enter...";
    std::cin.ignore();
  }
}

// Shows a simulation of a falling + exploding IIWA which "changes topology" by
// being rebuilt without joints.
TEST_F(TestWorkflows, ExplodingIiwaSim) {
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 1e-3);
  const std::string iiwa_file = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_spheres_dense_elbow_collision.urdf");
  Parser(&plant).AddModelFromFile(iiwa_file, "iiwa");
  // Add ground plane.
  auto X_FH = HalfSpace::MakePose({0, 0, 1}, {0, 0, 0});
  plant.RegisterCollisionGeometry(
      plant.world_body(), X_FH, HalfSpace(), "ground_plane_collision",
      CoulombFriction(0.8, 0.3));
  plant.Finalize();
  // Loosey-goosey - no control.
  for (const auto& model : GetModelInstances(plant)) {
    BuildWithNoControl(&builder, &plant, model);
  }
  geometry::DrakeVisualizerParams vis_params;
  vis_params.role = geometry::Role::kIllustration;
  if (FLAGS_visualize) {
    std::cout << "test_exploding_iiwa_sim" << std::endl;
    DrakeVisualizerd::AddToBuilder(&builder, scene_graph, nullptr, vis_params);
    ConnectContactResultsToDrakeVisualizer(&builder, plant, scene_graph);
  }
  auto diagram = builder.Build();
  // Set up context.
  auto d_context = diagram->CreateDefaultContext();
  auto &context = plant.GetMyMutableContextFromRoot(d_context.get());
  // - Hoist IIWA up in the air.
  plant.SetFreeBodyPose(&context, plant.GetBodyByName("base"),
                        RigidTransformd(Eigen::Vector3d(0, 0, 1.)));
  // - Set joint velocities to "spin" it in the air.
  for (const auto& joint : GetJoints(plant)) {
    if (auto result = CastsTo<const RevoluteJoint<double>*>(joint)) {
      result.DoIfCastable([&, &plant = plant](const auto* cast_joint_a) {
        Eigen::VectorXd pos(1);
        Eigen::VectorXd vel(1);
        pos << 0.7;
        vel << -5.;
        SetJointPositions(&plant, &context, *cast_joint_a, pos);
        SetJointVelocities(&plant, &context, *cast_joint_a, vel);
      });
    }
  }

  auto monitor = [&plant = plant](auto& mon_d_context) -> systems::EventStatus {
    // Stop the simulation once there's any contact.
    const auto &mon_context = plant.GetMyContextFromRoot(mon_d_context);
    const auto& query_object =
        plant.get_geometry_query_input_port()
            .Eval<geometry::QueryObject<double>>(mon_context);
    if (query_object.HasCollisions()) {
      return systems::EventStatus::ReachedTermination(&plant, "Contact");
    } else {
      return systems::EventStatus::DidNothing();
    }
  };

  // Forward simulate.
  Simulator<double> simulator(*diagram, std::move(d_context));
  simulator.Initialize();
  simulator.set_monitor(monitor);
  if (FLAGS_visualize) {
    simulator.set_target_realtime_rate(1.);
  }
  simulator.AdvanceTo(2.);
  // Try to push a bit further.
  simulator.clear_monitor();
  simulator.AdvanceTo(simulator.get_context().get_time() + 0.05);
  diagram->Publish(simulator.get_context());

  // Recreate simulator.
  DiagramBuilder<double> builder_new;
  auto [plant_new, scene_graph_new] = AddMultibodyPlantSceneGraph(
      &builder_new, plant.time_step());
  MultibodyPlantSubgraph subgraph(
      MultibodyPlantElements::FromPlant(&plant, &scene_graph));
  // Remove all joints; make them floating bodies.
  for (const auto &joint : GetJoints(plant)) {
    subgraph.RemoveJoint(joint);
  }
  // Remove massless bodies.
  // For more info, see: https://stackoverflow.com/a/62035705/7829525
  for (const auto& body : GetBodies(plant)) {
    if (body == &plant.world_body()) {
      continue;
    }
    const auto rigid_body = dynamic_cast<const RigidBody<double>*>(body);
    ASSERT_NE(rigid_body, nullptr);
    if (rigid_body->default_mass() == 0.) {
      subgraph.RemoveBody(body);
    }
  }
  // Finalize.
  auto to_new = subgraph.AddTo(&plant_new);
  plant_new.Finalize();
  if (FLAGS_visualize) {
    DrakeVisualizerd::AddToBuilder(&builder_new, scene_graph_new, nullptr,
                                   vis_params);
    ConnectContactResultsToDrakeVisualizer(&builder_new, plant_new,
                                           scene_graph_new);
  }
  auto diagram_new = builder_new.Build();
  // Remap state.
  auto d_context_new = diagram_new->CreateDefaultContext();
  d_context_new->SetTime(simulator.get_context().get_time());
  auto& context_new =
      plant_new.GetMyMutableContextFromRoot(d_context_new.get());
  to_new.CopyState(context, &context_new);
  // Simulate.
  Simulator simulator_new(*diagram_new, std::move(d_context_new));
  simulator_new.Initialize();
  diagram_new->Publish(simulator_new.get_context());
  if (FLAGS_visualize) {
    simulator_new.set_target_realtime_rate(1.);
  }
  simulator_new.AdvanceTo(context_new.get_time() + 2);
  if (FLAGS_visualize) {
    std::cout << "    Press enter...";
    std::cin.ignore();
  }
}

// TODO(azeey) Add DecompositionControllerLikeWorkflow test when SubgraphPolicy
// is implemented.
}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake

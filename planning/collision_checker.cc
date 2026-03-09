#include "drake/planning/collision_checker.h"

#include <algorithm>
#include <atomic>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/openmp_helpers.hpp>
#include <common_robotics_utilities/parallelism.hpp>
#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/utility.hpp>

#include "drake/common/drake_assert.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/text_logging.h"
#include "drake/planning/linear_distance_and_interpolation_provider.h"

namespace drake {
namespace planning {

using common_robotics_utilities::openmp_helpers::GetContextOmpThreadNum;
using common_robotics_utilities::parallelism::DegreeOfParallelism;
using common_robotics_utilities::parallelism::ParallelForBackend;
using common_robotics_utilities::parallelism::StaticParallelForIndexLoop;
using common_robotics_utilities::parallelism::StaticParallelForRangeLoop;
using common_robotics_utilities::parallelism::ThreadWorkRange;
using geometry::GeometryId;
using geometry::QueryObject;
using geometry::SceneGraphInspector;
using geometry::Shape;
using math::RigidTransform;
using multibody::BodyIndex;
using multibody::Frame;
using multibody::Joint;
using multibody::JointIndex;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::RigidBody;
using multibody::world_model_instance;
using systems::Context;

namespace {
// TODO(calderpg-tri, jwnimmer-tri) Remove unnecessary helpers once standalone
// distance and interpolation functions are no longer supported.
void SanityCheckConfigurationDistanceFunction(
    const ConfigurationDistanceFunction& distance_function,
    const Eigen::VectorXd& default_configuration) {
  const double test_distance =
      distance_function(default_configuration, default_configuration);
  DRAKE_THROW_UNLESS(test_distance == 0.0);
}

void SanityCheckConfigurationInterpolationFunction(
    const ConfigurationInterpolationFunction& interpolation_function,
    const Eigen::VectorXd& default_configuration) {
  const Eigen::VectorXd test_interpolated_q =
      interpolation_function(default_configuration, default_configuration, 0.0);
  DRAKE_THROW_UNLESS(test_interpolated_q.size() ==
                     default_configuration.size());
  for (int index = 0; index < test_interpolated_q.size(); ++index) {
    DRAKE_THROW_UNLESS(test_interpolated_q(index) ==
                       default_configuration(index));
  }
}

class LegacyDistanceAndInterpolationProvider final
    : public DistanceAndInterpolationProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LegacyDistanceAndInterpolationProvider);

  LegacyDistanceAndInterpolationProvider(
      const ConfigurationDistanceFunction& distance_function,
      const ConfigurationInterpolationFunction& interpolation_function)
      : distance_function_(distance_function),
        interpolation_function_(interpolation_function) {
    DRAKE_THROW_UNLESS(distance_function_ != nullptr);
    DRAKE_THROW_UNLESS(interpolation_function_ != nullptr);
  }

  std::shared_ptr<const LegacyDistanceAndInterpolationProvider>
  WithConfigurationDistanceFunction(
      const ConfigurationDistanceFunction& distance_function) const {
    return std::make_shared<LegacyDistanceAndInterpolationProvider>(
        distance_function, interpolation_function_);
  }

  std::shared_ptr<const LegacyDistanceAndInterpolationProvider>
  WithConfigurationInterpolationFunction(
      const ConfigurationInterpolationFunction& interpolation_function) const {
    return std::make_shared<LegacyDistanceAndInterpolationProvider>(
        distance_function_, interpolation_function);
  }

 private:
  double DoComputeConfigurationDistance(const Eigen::VectorXd& from,
                                        const Eigen::VectorXd& to) const final {
    return distance_function_(from, to);
  }

  Eigen::VectorXd DoInterpolateBetweenConfigurations(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to,
      double ratio) const final {
    return interpolation_function_(from, to, ratio);
  }

 private:
  const ConfigurationDistanceFunction distance_function_;
  const ConfigurationInterpolationFunction interpolation_function_;
};

// Default interpolator; it uses SLERP for quaternion-valued groups of dofs and
// LERP for everything else. See the documentation in CollisionChecker's
// edge checking group.
ConfigurationInterpolationFunction
MakeDefaultConfigurationInterpolationFunction(
    const std::vector<int>& quaternion_dof_start_indices) {
  return [quaternion_dof_start_indices](const Eigen::VectorXd& q_1,
                                        const Eigen::VectorXd& q_2,
                                        double ratio) {
    // Start with linear interpolation between q_1 and q_2.
    Eigen::VectorXd interpolated_q =
        common_robotics_utilities::math::InterpolateXd(q_1, q_2, ratio);
    // Handle quaternion dof properly.
    for (const int quat_dof_start_index : quaternion_dof_start_indices) {
      const Eigen::Quaterniond quat_1(q_1.segment<4>(quat_dof_start_index));
      const Eigen::Quaterniond quat_2(q_2.segment<4>(quat_dof_start_index));
      const Eigen::Quaterniond interpolated_quat =
          common_robotics_utilities::math::Interpolate(quat_1, quat_2, ratio);
      interpolated_q.segment<4>(quat_dof_start_index) =
          interpolated_quat.coeffs();
    }
    return interpolated_q;
  };
}

std::vector<int> GetQuaternionDofStartIndices(
    const MultibodyPlant<double>& plant) {
  return LinearDistanceAndInterpolationProvider(plant)
      .quaternion_dof_start_indices();
}

// Returns the set of indices in `q` that kinematically affect the robot model
// but that are not a part of the robot dofs (e.g., a floating or mobile base).
// This pre-computed analysis will be used for RobotClearance calculations.
// None of these indices are in qᵣ (the dofs controlled by the robot). It
// doesn't include all dofs that aren't in qᵣ, only those inboard of the robot.
std::vector<int> CalcUncontrolledDofsThatKinematicallyAffectTheRobot(
    const MultibodyPlant<double>& plant,
    const std::vector<ModelInstanceIndex>& robot) {
  // TODO(SeanCurtis-TRI): This algorithm was originally implemented to account
  // for floating bodies (bodies that had dofs that weren't controlled by
  // joints). Since #18390 all dofs are related to joints. We could rephrase
  // to determine the inboard, non-robot dofs directly, possibly simplifying
  // this function.

  // Our tactic in the loop below is to identify dofs that are either controlled
  // by one of our robot joints, or controlled by a non-robot joint but anyway
  // don't kinematically affect our robot. Then at the end, we'll return the
  // complement of this set.
  std::vector<const Joint<double>*> controlled_or_unaffecting;
  for (JointIndex joint_i : plant.GetJointIndices()) {
    const Joint<double>& joint = plant.get_joint(joint_i);
    // Skip these (or else GetBodiesKinematicallyAffectedBy will throw).
    if (joint.num_positions() == 0) {
      continue;
    }
    // Identify joints that are part of the robot.
    auto iter = std::find(robot.begin(), robot.end(), joint.model_instance());
    if (iter != robot.end()) {
      controlled_or_unaffecting.push_back(&joint);
      continue;
    }
    // Identify whether the joint affects any robot bodies.
    std::vector<BodyIndex> outboard_bodies =
        plant.GetBodiesKinematicallyAffectedBy({joint_i});
    bool affects_robot = false;
    for (const BodyIndex& body_i : outboard_bodies) {
      const RigidBody<double>& body = plant.get_body(body_i);
      iter = std::find(robot.begin(), robot.end(), body.model_instance());
      if (iter != robot.end()) {
        affects_robot = true;
        break;
      }
    }
    if (!affects_robot) {
      controlled_or_unaffecting.push_back(&joint);
    }
  }
  // Compute the list of controlled or unaffected indices in a position vector.
  using boolish = uint8_t;
  std::vector<boolish> controlled_or_unaffecting_array;
  controlled_or_unaffecting_array.resize(plant.num_positions(), false);
  for (const auto* joint : controlled_or_unaffecting) {
    const int start = joint->position_start();
    const int nq = joint->num_positions();
    for (int i = 0; i < nq; ++i) {
      controlled_or_unaffecting_array[start + i] = true;
    }
  }
  // Now return the complement: the uncontrolled & affecting indices.
  std::vector<int> result;
  for (int i = 0; i < plant.num_positions(); ++i) {
    if (!controlled_or_unaffecting_array[i]) {
      result.push_back(i);
    }
  }
  return result;
}

}  // namespace

CollisionChecker::~CollisionChecker() = default;

bool CollisionChecker::IsPartOfRobot(const RigidBody<double>& body) const {
  const ModelInstanceIndex needle = body.model_instance();
  const auto& haystack = robot_model_instances_;
  return std::binary_search(haystack.begin(), haystack.end(), needle);
}

bool CollisionChecker::IsPartOfRobot(BodyIndex body_index) const {
  return IsPartOfRobot(get_body(body_index));
}

const CollisionCheckerContext& CollisionChecker::model_context(
    const std::optional<int> context_number) const {
  const int context_index =
      context_number.has_value() ? *context_number : GetContextOmpThreadNum();
  return owned_contexts_.get_model_context(context_index);
}

std::shared_ptr<CollisionCheckerContext>
CollisionChecker::MakeStandaloneModelContext() const {
  // Make a shared clone of the special "prototype" context.
  std::shared_ptr<CollisionCheckerContext> standalone_context(
      owned_contexts_.prototype_context().Clone());
  // Save a weak reference to the shared standalone context.
  standalone_contexts_.AddStandaloneContext(standalone_context);
  return standalone_context;
}

void CollisionChecker::PerformOperationAgainstAllModelContexts(
    const std::function<void(const RobotDiagram<double>&,
                             CollisionCheckerContext*)>& operation) {
  DRAKE_THROW_UNLESS(operation != nullptr);
  owned_contexts_.PerformOperationAgainstAllOwnedContexts(model(), operation);
  standalone_contexts_.PerformOperationAgainstAllStandaloneContexts(  // BR
      model(), operation);
}

bool CollisionChecker::AddCollisionShape(
    const std::string& group_name, const BodyShapeDescription& description) {
  const RigidBody<double>& body = plant().GetBodyByName(
      description.body_name(),
      plant().GetModelInstanceByName(description.model_instance_name()));
  return AddCollisionShapeToBody(group_name, body, description.shape(),
                                 description.pose_in_body());
}

int CollisionChecker::AddCollisionShapes(
    const std::string& group_name,
    const std::vector<BodyShapeDescription>& descriptions) {
  int added = 0;
  for (const auto& body_shape : descriptions) {
    if (AddCollisionShape(group_name, body_shape)) {
      ++added;
    }
  }
  return added;
}

std::map<std::string, int> CollisionChecker::AddCollisionShapes(
    const std::map<std::string, std::vector<BodyShapeDescription>>&
        geometry_groups) {
  std::map<std::string, int> group_added_shapes;
  for (const auto& [group_name, group_shapes] : geometry_groups) {
    const int num_added_shapes = AddCollisionShapes(group_name, group_shapes);
    group_added_shapes.emplace(group_name, num_added_shapes);
  }
  return group_added_shapes;
}

bool CollisionChecker::AddCollisionShapeToFrame(
    const std::string& group_name, const Frame<double>& frameA,
    const Shape& shape, const RigidTransform<double>& X_AG) {
  const RigidBody<double>& bodyA = frameA.body();
  const RigidTransform<double>& X_BA = frameA.GetFixedPoseInBodyFrame();
  const RigidTransform<double> X_BG = X_BA * X_AG;
  return AddCollisionShapeToBody(group_name, bodyA, shape, X_BG);
}

bool CollisionChecker::AddCollisionShapeToBody(
    const std::string& group_name, const RigidBody<double>& bodyA,
    const Shape& shape, const RigidTransform<double>& X_AG) {
  const std::optional<GeometryId> maybe_geometry =
      DoAddCollisionShapeToBody(group_name, bodyA, shape, X_AG);
  if (maybe_geometry.has_value()) {
    const std::string& model_instance_name =
        plant().GetModelInstanceName(bodyA.model_instance());
    geometry_groups_[group_name].push_back(AddedShape{
        *maybe_geometry, bodyA.index(),
        BodyShapeDescription(shape, X_AG, model_instance_name, bodyA.name())});
  }
  return maybe_geometry.has_value();
}

std::map<std::string, std::vector<BodyShapeDescription>>
CollisionChecker::GetAllAddedCollisionShapes() const {
  std::map<std::string, std::vector<BodyShapeDescription>> result;
  for (const auto& [group_name, group_shapes] : geometry_groups_) {
    result[group_name].reserve(group_shapes.size());
    for (const auto& shape : group_shapes) {
      result[group_name].push_back(shape.description);
    }
  }
  return result;
}

void CollisionChecker::RemoveAllAddedCollisionShapes(
    const std::string& group_name) {
  auto iter = geometry_groups_.find(group_name);
  if (iter != geometry_groups_.end()) {
    drake::log()->debug("Removing geometries from group [{}].", group_name);
    RemoveAddedGeometries(iter->second);
    geometry_groups_.erase(iter);
  }
}

void CollisionChecker::RemoveAllAddedCollisionShapes() {
  drake::log()->debug("Removing all added geometries");
  for (const auto& [group_name, group_ids] : geometry_groups_) {
    RemoveAddedGeometries(group_ids);
  }
  geometry_groups_.clear();
}

std::optional<double> CollisionChecker::MaybeGetUniformRobotEnvironmentPadding()
    const {
  // TODO(SeanCurtis-TRI): We have three functions that walk a triangular
  // portion of the padding matrix. Consider unifying the logic for the
  // triangular indices.
  std::optional<double> check_padding;
  for (BodyIndex body_index(0); body_index < plant().num_bodies();
       ++body_index) {
    for (BodyIndex other_body_index(body_index + 1);
         other_body_index < plant().num_bodies(); ++other_body_index) {
      if (IsPartOfRobot(get_body(body_index)) !=
          IsPartOfRobot(get_body(other_body_index))) {
        const double this_padding =
            GetPaddingBetween(body_index, other_body_index);
        if (!check_padding.has_value()) {
          check_padding = this_padding;
        }
        if (check_padding.value() != this_padding) {
          return std::nullopt;
        }
      }
    }
  }
  return check_padding;
}

std::optional<double> CollisionChecker::MaybeGetUniformRobotRobotPadding()
    const {
  std::optional<double> check_padding;
  for (BodyIndex body_index(0); body_index < plant().num_bodies();
       ++body_index) {
    for (BodyIndex other_body_index(body_index + 1);
         other_body_index < plant().num_bodies(); ++other_body_index) {
      if (IsPartOfRobot(get_body(body_index)) &&
          IsPartOfRobot(get_body(other_body_index))) {
        const double this_padding =
            GetPaddingBetween(body_index, other_body_index);
        if (!check_padding.has_value()) {
          check_padding = this_padding;
        }
        if (check_padding.value() != this_padding) {
          return std::nullopt;
        }
      }
    }
  }
  return check_padding;
}

void CollisionChecker::SetPaddingBetween(BodyIndex bodyA_index,
                                         BodyIndex bodyB_index,
                                         double padding) {
  DRAKE_THROW_UNLESS(bodyA_index >= 0 &&
                     bodyA_index < collision_padding_.rows());
  DRAKE_THROW_UNLESS(bodyB_index >= 0 &&
                     bodyB_index < collision_padding_.rows());
  DRAKE_THROW_UNLESS(bodyA_index != bodyB_index);
  DRAKE_THROW_UNLESS(std::isfinite(padding));
  DRAKE_THROW_UNLESS(IsPartOfRobot(get_body(bodyA_index)) ||
                     IsPartOfRobot(get_body(bodyB_index)));
  collision_padding_(int{bodyA_index}, int{bodyB_index}) = padding;
  collision_padding_(int{bodyB_index}, int{bodyA_index}) = padding;
  UpdateMaxCollisionPadding();
}

void CollisionChecker::SetPaddingMatrix(
    const Eigen::MatrixXd& requested_padding) {
  // First confirm it is of appropriate *size*.
  if (requested_padding.rows() != collision_padding_.rows() ||
      requested_padding.cols() != collision_padding_.cols()) {
    throw std::logic_error(
        fmt::format("CollisionChecker::SetPaddingMatrix(): The padding"
                    " matrix must be {}x{}. The given padding matrix is the"
                    " wrong size: {}x{}.",
                    collision_padding_.rows(), collision_padding_.cols(),
                    requested_padding.rows(), requested_padding.cols()));
  }
  ValidatePaddingMatrix(requested_padding, __func__);
  collision_padding_ = requested_padding;
  UpdateMaxCollisionPadding();
}

void CollisionChecker::SetPaddingOneRobotBodyAllEnvironmentPairs(
    const BodyIndex body_index, const double padding) {
  DRAKE_THROW_UNLESS(std::isfinite(padding));
  DRAKE_THROW_UNLESS(IsPartOfRobot(get_body(body_index)));
  for (BodyIndex other_body_index(0); other_body_index < plant().num_bodies();
       ++other_body_index) {
    if (!IsPartOfRobot(get_body(other_body_index))) {
      collision_padding_(int{body_index}, int{other_body_index}) = padding;
      collision_padding_(int{other_body_index}, int{body_index}) = padding;
    }
  }
  UpdateMaxCollisionPadding();
}

void CollisionChecker::SetPaddingAllRobotEnvironmentPairs(
    const double padding) {
  DRAKE_THROW_UNLESS(std::isfinite(padding));
  for (BodyIndex body_index(0); body_index < plant().num_bodies();
       ++body_index) {
    for (BodyIndex other_body_index(body_index + 1);
         other_body_index < plant().num_bodies(); ++other_body_index) {
      if (IsPartOfRobot(get_body(body_index)) !=
          IsPartOfRobot(get_body(other_body_index))) {
        collision_padding_(int{body_index}, int{other_body_index}) = padding;
        collision_padding_(int{other_body_index}, int{body_index}) = padding;
      }
    }
  }
  UpdateMaxCollisionPadding();
}

void CollisionChecker::SetPaddingAllRobotRobotPairs(const double padding) {
  DRAKE_THROW_UNLESS(std::isfinite(padding));
  for (BodyIndex body_index(0); body_index < plant().num_bodies();
       ++body_index) {
    for (BodyIndex other_body_index(body_index + 1);
         other_body_index < plant().num_bodies(); ++other_body_index) {
      if (IsPartOfRobot(get_body(body_index)) &&
          IsPartOfRobot(get_body(other_body_index))) {
        collision_padding_(int{body_index}, int{other_body_index}) = padding;
        collision_padding_(int{other_body_index}, int{body_index}) = padding;
      }
    }
  }
  UpdateMaxCollisionPadding();
}

void CollisionChecker::SetCollisionFilterMatrix(
    const Eigen::MatrixXi& filter_matrix) {
  // First confirm it is of appropriate *size*.
  if (filter_matrix.rows() != filtered_collisions_.rows() ||
      filter_matrix.cols() != filtered_collisions_.cols()) {
    throw std::logic_error(
        fmt::format("CollisionChecker::SetCollisionFilterMatrix(): The filter "
                    "matrix must be {}x{};. The given matrix is the wrong "
                    "size: {}x{}.",
                    filtered_collisions_.rows(), filtered_collisions_.cols(),
                    filter_matrix.rows(), filter_matrix.cols()));
  }
  // Only perform additional work if the provided filter matrix is different
  // from the current matrix.
  if (filtered_collisions_ != filter_matrix) {
    // Now test for consistency.
    ValidateFilteredCollisionMatrix(filter_matrix, __func__);
    filtered_collisions_ = filter_matrix;
    // Allow derived checkers to perform any post-filter-change work.
    UpdateCollisionFilters();
  }
}

bool CollisionChecker::IsCollisionFilteredBetween(BodyIndex bodyA_index,
                                                  BodyIndex bodyB_index) const {
  DRAKE_THROW_UNLESS(bodyA_index >= 0 &&
                     bodyA_index < filtered_collisions_.rows());
  DRAKE_THROW_UNLESS(bodyB_index >= 0 &&
                     bodyB_index < filtered_collisions_.rows());
  return (filtered_collisions_(int{bodyA_index}, int{bodyB_index}) != 0);
}

void CollisionChecker::SetCollisionFilteredBetween(BodyIndex bodyA_index,
                                                   BodyIndex bodyB_index,
                                                   bool filter_collision) {
  const int N = filtered_collisions_.rows();
  DRAKE_THROW_UNLESS(bodyA_index >= 0 && bodyA_index < N);
  DRAKE_THROW_UNLESS(bodyB_index >= 0 && bodyB_index < N);
  DRAKE_THROW_UNLESS(bodyA_index != bodyB_index);
  if (!(IsPartOfRobot(bodyA_index) || IsPartOfRobot(bodyB_index))) {
    throw std::logic_error(
        fmt::format("CollisionChecker::SetCollisionFilteredBetween(): cannot "
                    "be used on pairs of environment bodies: ({}, {})",
                    bodyA_index, bodyB_index));
  }
  const int current_value =
      filtered_collisions_(int{bodyA_index}, int{bodyB_index});
  const int new_value = filter_collision ? 1 : 0;
  // Only perform additional work if the specified filter will change the filter
  // matrix.
  if (new_value != current_value) {
    // The tests above should mean that we're not trying to write to an entry
    // that is locked in a filtered state (-1); just in case, we'll add one more
    // explicit test.
    DRAKE_ASSERT(current_value != -1);
    filtered_collisions_(int{bodyA_index}, int{bodyB_index}) = new_value;
    filtered_collisions_(int{bodyB_index}, int{bodyA_index}) = new_value;
    // Allow derived checkers to perform any post-filter-change work.
    UpdateCollisionFilters();
  }
}

void CollisionChecker::SetCollisionFilteredWithAllBodies(BodyIndex body_index) {
  DRAKE_THROW_UNLESS(body_index >= 0 &&
                     body_index < filtered_collisions_.rows());
  DRAKE_THROW_UNLESS(IsPartOfRobot(body_index));
  const Eigen::MatrixXi prior_filter_matrix = filtered_collisions_;
  filtered_collisions_.row(body_index).setConstant(1);
  filtered_collisions_.col(body_index).setConstant(1);
  // Maintain the invariant that the diagonal is always -1.
  filtered_collisions_(int{body_index}, int{body_index}) = -1;
  // Only perform additional work if the filter matrix has changed.
  if (prior_filter_matrix != filtered_collisions_) {
    // Allow derived checkers to perform any post-filter-change work.
    UpdateCollisionFilters();
  }
}

bool CollisionChecker::CheckConfigCollisionFree(
    const Eigen::VectorXd& q, const std::optional<int> context_number) const {
  return CheckContextConfigCollisionFree(&mutable_model_context(context_number),
                                         q);
}

bool CollisionChecker::CheckContextConfigCollisionFree(
    CollisionCheckerContext* model_context, const Eigen::VectorXd& q) const {
  DRAKE_THROW_UNLESS(model_context != nullptr);
  UpdateContextPositions(model_context, q);
  return DoCheckContextConfigCollisionFree(*model_context);
}

std::vector<uint8_t> CollisionChecker::CheckConfigsCollisionFree(
    const std::vector<Eigen::VectorXd>& configs,
    const Parallelism parallelize) const {
  // Note: vector<uint8_t> is used since vector<bool> is not thread safe.
  std::vector<uint8_t> collision_checks(configs.size(), 0);

  const int number_of_threads = GetNumberOfThreads(parallelize);
  drake::log()->debug("CheckConfigsCollisionFree uses {} thread(s)",
                      number_of_threads);

  const auto config_work = [&](const int thread_num, const int64_t index) {
    collision_checks.at(index) =
        CheckConfigCollisionFree(configs.at(index), thread_num);
  };

  StaticParallelForIndexLoop(DegreeOfParallelism(number_of_threads), 0,
                             configs.size(), config_work,
                             ParallelForBackend::BEST_AVAILABLE);

  return collision_checks;
}

void CollisionChecker::SetDistanceAndInterpolationProvider(
    std::shared_ptr<const DistanceAndInterpolationProvider> provider) {
  DRAKE_THROW_UNLESS(provider != nullptr);

  const Eigen::VectorXd& default_q = GetDefaultConfiguration();

  const double test_distance =
      provider->ComputeConfigurationDistance(default_q, default_q);
  DRAKE_THROW_UNLESS(test_distance == 0.0);

  const Eigen::VectorXd test_interpolated_q =
      provider->InterpolateBetweenConfigurations(default_q, default_q, 0.0);
  DRAKE_THROW_UNLESS(test_interpolated_q.size() == default_q.size());
  for (int index = 0; index < test_interpolated_q.size(); ++index) {
    DRAKE_THROW_UNLESS(test_interpolated_q(index) == default_q(index));
  }

  distance_and_interpolation_provider_ = std::move(provider);
}

void CollisionChecker::SetConfigurationDistanceFunction(
    const ConfigurationDistanceFunction& distance_function) {
  auto legacy =
      std::dynamic_pointer_cast<const LegacyDistanceAndInterpolationProvider>(
          distance_and_interpolation_provider_);
  if (legacy == nullptr) {
    throw std::logic_error(
        "CollisionChecker::SetConfigurationDistanceFunction() "
        "is not supported after a DistanceAndInterpolationProvider "
        "has already been set.");
  }
  DRAKE_THROW_UNLESS(distance_function != nullptr);
  SanityCheckConfigurationDistanceFunction(distance_function,
                                           GetDefaultConfiguration());
  distance_and_interpolation_provider_ =
      legacy->WithConfigurationDistanceFunction(distance_function);
}

ConfigurationDistanceFunction
CollisionChecker::MakeStandaloneConfigurationDistanceFunction() const {
  return [this](const Eigen::VectorXd& q_1, const Eigen::VectorXd& q_2) {
    return this->ComputeConfigurationDistance(q_1, q_2);
  };
}

void CollisionChecker::SetConfigurationInterpolationFunction(
    const ConfigurationInterpolationFunction& interpolation_function) {
  auto legacy =
      std::dynamic_pointer_cast<const LegacyDistanceAndInterpolationProvider>(
          distance_and_interpolation_provider_);
  if (legacy == nullptr) {
    throw std::logic_error(
        "CollisionChecker::SetConfigurationInterpolationFunction() "
        "is not supported after a DistanceAndInterpolationProvider "
        "has already been set.");
  }
  if (interpolation_function == nullptr) {
    SetConfigurationInterpolationFunction(
        MakeDefaultConfigurationInterpolationFunction(
            GetQuaternionDofStartIndices(plant())));
    return;
  }
  SanityCheckConfigurationInterpolationFunction(interpolation_function,
                                                GetDefaultConfiguration());
  distance_and_interpolation_provider_ =
      legacy->WithConfigurationInterpolationFunction(interpolation_function);
}

ConfigurationInterpolationFunction
CollisionChecker::MakeStandaloneConfigurationInterpolationFunction() const {
  return [this](const Eigen::VectorXd& q_1, const Eigen::VectorXd& q_2,
                double ratio) {
    return this->InterpolateBetweenConfigurations(q_1, q_2, ratio);
  };
}

bool CollisionChecker::CheckEdgeCollisionFree(
    const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
    const std::optional<int> context_number) const {
  return CheckContextEdgeCollisionFree(&mutable_model_context(context_number),
                                       q1, q2);
}

bool CollisionChecker::CheckContextEdgeCollisionFree(
    CollisionCheckerContext* model_context, const Eigen::VectorXd& q1,
    const Eigen::VectorXd& q2) const {
  DRAKE_THROW_UNLESS(model_context != nullptr);

  // Fail fast if q2 is in collision. This method is used by motion planners
  // that extend/connect towards some target configuration, and thus require a
  // number of edge collision checks in which q1 is often known to be
  // collision-free while q2 is unknown. Many of these potential
  // extensions/connections will result in a colliding configuration, so failing
  // fast on a colliding q2 helps reduce the work of checking colliding edges.
  // There is also no need to special case checking q1, since it will be the
  // first configuration checked in the loop.
  if (!CheckContextConfigCollisionFree(model_context, q2)) {
    // Checking q2 throws if q2 contains non-finite values.
    // However, if q2 is all finite and in collision, we still should throw if
    // q1 isn't finite; the return value is reserved for valid inputs.
    DRAKE_THROW_UNLESS(q1.allFinite());
    return false;
  }

  const double distance = ComputeConfigurationDistance(q1, q2);
  const int num_steps =
      static_cast<int>(std::max(1.0, std::ceil(distance / edge_step_size())));
  for (int step = 0; step < num_steps; ++step) {
    const double ratio =
        static_cast<double>(step) / static_cast<double>(num_steps);
    const Eigen::VectorXd qinterp =
        InterpolateBetweenConfigurations(q1, q2, ratio);
    if (!CheckContextConfigCollisionFree(model_context, qinterp)) {
      return false;
    }
  }
  return true;
}

bool CollisionChecker::CheckEdgeCollisionFreeParallel(
    const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
    const Parallelism parallelize) const {
  const int number_of_threads = GetNumberOfThreads(parallelize);
  drake::log()->debug("CheckEdgeCollisionFreeParallel uses {} thread(s)",
                      number_of_threads);

  // Only perform parallel operations if `omp parallel for` will use >1 thread.
  if (number_of_threads > 1) {
    // Fail fast if q2 is in collision. This method is used by motion planners
    // that extend/connect towards some target configuration, and thus require a
    // number of edge collision checks in which q1 is often known to be
    // collision-free while q2 is unknown. Many of these potential
    // extensions/connections will result in a colliding configuration, so
    // failing fast on a colliding q2 helps reduce the work of checking
    // colliding edges.
    if (!CheckConfigCollisionFree(q2)) {
      // Checking q2 throws if q2 contains non-finite values.
      // However, if q2 is all finite and in collision, we still should throw if
      // q1 isn't finite; the return value is reserved for valid inputs.
      DRAKE_THROW_UNLESS(q1.allFinite());
      return false;
    }
    // Special case q1 as well, so it gets checked before parallel dispatch.
    if (!CheckConfigCollisionFree(q1)) {
      return false;
    }

    const double distance = ComputeConfigurationDistance(q1, q2);
    const int num_steps =
        static_cast<int>(std::max(1.0, std::ceil(distance / edge_step_size())));
    std::atomic<bool> edge_valid(true);

    const auto step_range_work = [&](const ThreadWorkRange& work_range) {
      for (int64_t step = work_range.GetRangeStart();
           step < work_range.GetRangeEnd(); ++step) {
        if (edge_valid.load()) {
          // If no collision has been found, check the next step.
          const double ratio =
              static_cast<double>(step) / static_cast<double>(num_steps);
          const Eigen::VectorXd qinterp =
              InterpolateBetweenConfigurations(q1, q2, ratio);
          if (!CheckConfigCollisionFree(qinterp, work_range.GetThreadNum())) {
            // If we have encountered a collision, record it and end early.
            edge_valid.store(false);
            break;
          }
        } else {
          // If another thread encountered a collision, end early as well.
          break;
        }
      }
    };

    // Note that the range starts at 1, not 0, as we have already checked q1.
    StaticParallelForRangeLoop(DegreeOfParallelism(number_of_threads), 1,
                               num_steps, step_range_work,
                               ParallelForBackend::BEST_AVAILABLE);

    return edge_valid.load();
  } else {
    // If OpenMP cannot parallelize, fall back to the serial version.
    return CheckEdgeCollisionFree(q1, q2);
  }
}

std::vector<uint8_t> CollisionChecker::CheckEdgesCollisionFree(
    const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges,
    const Parallelism parallelize) const {
  // Note: vector<uint8_t> is used since vector<bool> is not thread safe.
  std::vector<uint8_t> collision_checks(edges.size(), 0);

  const int number_of_threads = GetNumberOfThreads(parallelize);
  drake::log()->debug("CheckEdgesCollisionFree uses {} thread(s)",
                      number_of_threads);

  const auto edge_work = [&](const int thread_num, const int64_t index) {
    const std::pair<Eigen::VectorXd, Eigen::VectorXd>& edge = edges.at(index);

    collision_checks.at(index) =
        CheckEdgeCollisionFree(edge.first, edge.second, thread_num);
  };

  StaticParallelForIndexLoop(DegreeOfParallelism(number_of_threads), 0,
                             edges.size(), edge_work,
                             ParallelForBackend::BEST_AVAILABLE);

  return collision_checks;
}

EdgeMeasure CollisionChecker::MeasureEdgeCollisionFree(
    const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
    const std::optional<int> context_number) const {
  return MeasureContextEdgeCollisionFree(&mutable_model_context(context_number),
                                         q1, q2);
}

EdgeMeasure CollisionChecker::MeasureContextEdgeCollisionFree(
    CollisionCheckerContext* model_context, const Eigen::VectorXd& q1,
    const Eigen::VectorXd& q2) const {
  DRAKE_THROW_UNLESS(model_context != nullptr);
  const double distance = ComputeConfigurationDistance(q1, q2);
  const int num_steps =
      static_cast<int>(std::max(1.0, std::ceil(distance / edge_step_size())));
  double last_valid_ratio = -1.0;
  for (int step = 0; step <= num_steps; ++step) {
    const double ratio =
        static_cast<double>(step) / static_cast<double>(num_steps);
    const Eigen::VectorXd qinterp =
        InterpolateBetweenConfigurations(q1, q2, ratio);
    if (!CheckContextConfigCollisionFree(model_context, qinterp)) {
      return EdgeMeasure(distance, last_valid_ratio);
    }
    last_valid_ratio = ratio;
  }
  return EdgeMeasure(distance, 1.0);
}

EdgeMeasure CollisionChecker::MeasureEdgeCollisionFreeParallel(
    const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
    const Parallelism parallelize) const {
  const int number_of_threads = GetNumberOfThreads(parallelize);
  drake::log()->debug("MeasureEdgeCollisionFreeParallel uses {} thread(s)",
                      number_of_threads);

  // Only perform parallel operations if `omp parallel for` will use >1 thread.
  if (number_of_threads > 1) {
    const double distance = ComputeConfigurationDistance(q1, q2);
    const int num_steps =
        static_cast<int>(std::max(1.0, std::ceil(distance / edge_step_size())));
    // The "highest" interpolant value, alpha, (uninterrupted from q1) for
    // which there is no collision.
    std::atomic<double> alpha;
    // Start by assuming the whole edge is fine; we'll whittle away at it.
    alpha.store(1.0);

    // This is a manual implementation of the core behavior of
    // std::atomic<T>::fetch_min from C++26, and can be replaced with fetch_min
    // once available.
    const auto fetch_min = [](std::atomic<double>* existing, double value) {
      double stored = existing->load();
      while (value < stored) {
        if (existing->compare_exchange_weak(stored, value)) {
          break;
        }
      }
    };

    const auto step_range_work = [&](const ThreadWorkRange& work_range) {
      for (int64_t step = work_range.GetRangeStart();
           step < work_range.GetRangeEnd(); ++step) {
        const double ratio =
            static_cast<double>(step) / static_cast<double>(num_steps);
        // If this step fails, this is the alpha which we would report.
        const double possible_alpha =
            static_cast<double>(step - 1) / static_cast<double>(num_steps);

        if (possible_alpha < alpha.load()) {
          // If no lower alpha has been found, check the next configuration.
          const Eigen::VectorXd qinterp =
              InterpolateBetweenConfigurations(q1, q2, ratio);
          if (!CheckConfigCollisionFree(qinterp, work_range.GetThreadNum())) {
            // Between the initial decision to interpolate and check collisions
            // and now, another thread may have proven a *lower* alpha is
            // invalid; we need an atomic min of our current alpha and the
            // stored alpha.
            fetch_min(&alpha, possible_alpha);
            // Stop because we have found a colliding configuration.
            break;
          }
        } else {
          // A different thread has already found a lower alpha, so we can stop.
          break;
        }
      }
    };

    StaticParallelForRangeLoop(DegreeOfParallelism(number_of_threads), 0,
                               num_steps + 1, step_range_work,
                               ParallelForBackend::BEST_AVAILABLE);

    return EdgeMeasure(distance, alpha.load());
  } else {
    // If OpenMP cannot parallelize, fall back to the serial version.
    return MeasureEdgeCollisionFree(q1, q2);
  }
}

std::vector<EdgeMeasure> CollisionChecker::MeasureEdgesCollisionFree(
    const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges,
    const Parallelism parallelize) const {
  std::vector<EdgeMeasure> collision_checks(edges.size(),
                                            EdgeMeasure(0.0, -1.0));

  const int number_of_threads = GetNumberOfThreads(parallelize);
  drake::log()->debug("MeasureEdgesCollisionFree uses {} thread(s)",
                      number_of_threads);

  const auto edge_work = [&](const int thread_num, const int64_t index) {
    const std::pair<Eigen::VectorXd, Eigen::VectorXd>& edge = edges.at(index);

    collision_checks.at(index) =
        MeasureEdgeCollisionFree(edge.first, edge.second, thread_num);
  };

  StaticParallelForIndexLoop(DegreeOfParallelism(number_of_threads), 0,
                             edges.size(), edge_work,
                             ParallelForBackend::BEST_AVAILABLE);

  return collision_checks;
}

RobotClearance CollisionChecker::CalcRobotClearance(
    const Eigen::VectorXd& q, const double influence_distance,
    const std::optional<int> context_number) const {
  return CalcContextRobotClearance(&mutable_model_context(context_number), q,
                                   influence_distance);
}

RobotClearance CollisionChecker::CalcContextRobotClearance(
    CollisionCheckerContext* model_context, const Eigen::VectorXd& q,
    const double influence_distance) const {
  DRAKE_THROW_UNLESS(model_context != nullptr);
  DRAKE_THROW_UNLESS(influence_distance >= 0.0);
  DRAKE_THROW_UNLESS(std::isfinite(influence_distance));
  UpdateContextPositions(model_context, q);
  auto result = DoCalcContextRobotClearance(*model_context, influence_distance);
  for (int j : uncontrolled_dofs_that_kinematically_affect_the_robot_) {
    result.mutable_jacobians().col(j).setZero();
  }
  return result;
}

int CollisionChecker::MaxNumDistances(
    const std::optional<int> context_number) const {
  return MaxContextNumDistances(model_context(context_number));
}

int CollisionChecker::MaxContextNumDistances(
    const CollisionCheckerContext& model_context) const {
  return DoMaxContextNumDistances(model_context);
}

std::vector<RobotCollisionType> CollisionChecker::ClassifyBodyCollisions(
    const Eigen::VectorXd& q, const std::optional<int> context_number) const {
  return ClassifyContextBodyCollisions(&mutable_model_context(context_number),
                                       q);
}

std::vector<RobotCollisionType> CollisionChecker::ClassifyContextBodyCollisions(
    CollisionCheckerContext* model_context, const Eigen::VectorXd& q) const {
  DRAKE_THROW_UNLESS(model_context != nullptr);
  UpdateContextPositions(model_context, q);
  return DoClassifyContextBodyCollisions(*model_context);
}

CollisionChecker::CollisionChecker(CollisionCheckerParams params,
                                   bool supports_parallel_checking)
    : setup_model_(std::move(params.model)),
      robot_model_instances_([&params]() {
        // Sort (and de-duplicate) the robot model instances for faster lookups.
        DRAKE_THROW_UNLESS(params.robot_model_instances.size() > 0);
        const std::set<ModelInstanceIndex> sorted_set(
            params.robot_model_instances.begin(),
            params.robot_model_instances.end());
        const std::vector<ModelInstanceIndex> sorted_vec(sorted_set.begin(),
                                                         sorted_set.end());
        const ModelInstanceIndex world = world_model_instance();
        for (const auto& robot_model_instance : sorted_vec) {
          DRAKE_THROW_UNLESS(robot_model_instance != world);
        }
        return sorted_vec;
      }()),
      uncontrolled_dofs_that_kinematically_affect_the_robot_(
          CalcUncontrolledDofsThatKinematicallyAffectTheRobot(
              setup_model_->plant(), robot_model_instances_)),
      supports_parallel_checking_(supports_parallel_checking),
      implicit_context_parallelism_(params.implicit_context_parallelism) {
  // Sanity check the supported implicit context parallelism.
  if (!SupportsParallelChecking() &&
      implicit_context_parallelism_.num_threads() > 1) {
    throw std::runtime_error(
        "implicit context parallelism > 1 cannot be used with a collision "
        "checker that does not support parallel operations");
  }

  // Initialize the zero configuration.
  zero_configuration_ = Eigen::VectorXd::Zero(plant().num_positions());
  // Initialize the default configuration.
  default_configuration_ =
      plant().GetPositions(*plant().CreateDefaultContext());

  // Initialize the collision padding matrix.
  collision_padding_ =
      Eigen::MatrixXd::Zero(plant().num_bodies(), plant().num_bodies());
  SetPaddingAllRobotEnvironmentPairs(params.env_collision_padding);
  SetPaddingAllRobotRobotPairs(params.self_collision_padding);

  // Set distance and interpolation provider/functions.
  const bool params_has_provider =
      params.distance_and_interpolation_provider != nullptr;
  const bool params_has_distance_function =
      params.configuration_distance_function != nullptr;

  if (params_has_provider && params_has_distance_function) {
    throw std::runtime_error(
        "CollisionCheckerParams may contain either "
        "distance_and_interpolation_provider != nullptr "
        "or distance_function != nullptr, not both");
  }

  if (params_has_provider) {
    SetDistanceAndInterpolationProvider(
        std::move(params.distance_and_interpolation_provider));
  } else if (params_has_distance_function) {
    SanityCheckConfigurationDistanceFunction(
        params.configuration_distance_function, GetDefaultConfiguration());
    // Generate the default interpolation function.
    const ConfigurationInterpolationFunction default_interpolation_fn =
        MakeDefaultConfigurationInterpolationFunction(
            GetQuaternionDofStartIndices(plant()));
    distance_and_interpolation_provider_ =
        std::make_unique<LegacyDistanceAndInterpolationProvider>(
            params.configuration_distance_function, default_interpolation_fn);
  } else {
    SetDistanceAndInterpolationProvider(
        std::make_unique<LinearDistanceAndInterpolationProvider>(plant()));
  }

  // Set edge step size.
  set_edge_step_size(params.edge_step_size);

  // Generate the filtered collision matrix.
  nominal_filtered_collisions_ = GenerateFilteredCollisionMatrix();
  filtered_collisions_ = nominal_filtered_collisions_;
  log()->debug("Collision filter matrix:\n{}", fmt_eigen(filtered_collisions_));
}

CollisionChecker::CollisionChecker(const CollisionChecker&) = default;

void CollisionChecker::AllocateContexts() {
  DRAKE_THROW_UNLESS(IsInitialSetup());
  // Move to a const model.
  model_ = std::move(setup_model_);
  // Make enough contexts to support the specified implicit context parallelism.
  log()->info("Allocating contexts to support implicit context parallelism {}",
              implicit_context_parallelism_.num_threads());
  // Make the prototype context.
  const std::unique_ptr<CollisionCheckerContext> prototype_context =
      CreatePrototypeContext();
  DRAKE_THROW_UNLESS(prototype_context != nullptr);
  owned_contexts_.AllocateOwnedContexts(
      *prototype_context, implicit_context_parallelism_.num_threads());
}

void CollisionChecker::OwnedContextKeeper::AllocateOwnedContexts(
    const CollisionCheckerContext& prototype_context, const int num_contexts) {
  DRAKE_THROW_UNLESS(num_contexts >= 1);
  DRAKE_THROW_UNLESS(empty());
  for (int index = 0; index < num_contexts; ++index) {
    auto cloned = prototype_context.Clone();
    // Enforce the invariant that contexts are non-null.
    DRAKE_THROW_UNLESS(cloned != nullptr);
    model_contexts_.emplace_back(std::move(cloned));
  }
  prototype_context_ = prototype_context.Clone();
  DRAKE_THROW_UNLESS(prototype_context_ != nullptr);
}

bool CollisionChecker::CanEvaluateInParallel() const {
  return SupportsParallelChecking() && num_allocated_contexts() > 1;
}

std::string CollisionChecker::CriticizePaddingMatrix() const {
  return CriticizePaddingMatrix(collision_padding_, __func__);
}

CollisionCheckerContext& CollisionChecker::mutable_model_context(
    const std::optional<int> context_number) const {
  const int context_index =
      context_number.has_value() ? *context_number : GetContextOmpThreadNum();
  return owned_contexts_.get_mutable_model_context(context_index);
}

void CollisionChecker::ValidateFilteredCollisionMatrix(
    const Eigen::MatrixXi& filtered, const char* func) const {
  DRAKE_THROW_UNLESS(filtered.rows() == filtered.cols());
  const int count = filtered.rows();
  // TODO(sean.curtis): These messages have the prefix CollisionChecker.
  // Consider whether that should be replaced with NiceTypeName::Get(*this).
  // Loop counters below use `int` for Eigen indexing compatibility.
  for (int i = 0; i < count; ++i) {
    if (filtered(i, i) != -1) {
      throw std::logic_error(fmt::format(
          "CollisionChecker::{}(): The filtered collision matrix has invalid "
          "values on the diagonal ({}, {}) = {}; the values on the diagonal "
          "must always be -1.",
          func, i, i, filtered(i, i)));
    }

    const bool i_is_robot = IsPartOfRobot(BodyIndex(i));
    for (int j = i + 1; j < count; ++j) {
      const bool pair_has_robot = i_is_robot || IsPartOfRobot(BodyIndex(j));
      // Both are environment bodies.
      if (!pair_has_robot && filtered(i, j) != -1) {
        throw std::logic_error(fmt::format(
            "CollisionChecker::{}(): The filtered collision matrix must "
            "contain -1 for pairs of environment bodies. Found {} at ({}, {}).",
            func, filtered(i, j), i, j));
      }
      // Values must lie in the range [-1, 1].
      if (filtered(i, j) > 1 || filtered(i, j) < -1) {
        throw std::logic_error(fmt::format(
            "CollisionChecker::{}(): The filtered collision matrix must "
            "contain values that are 0, 1, or -1. Found {} at ({}, {}).",
            func, filtered(i, j), i, j));
      }
      // Matrix must be symmetric.
      if (filtered(i, j) != filtered(j, i)) {
        throw std::logic_error(fmt::format(
            "CollisionChecker::{}(): The filtered collision matrix must be "
            "symmetric. Values at ({}, {}) and ({}, {}) are not equal; "
            "{} != {}.",
            func, i, j, j, i, filtered(i, j), filtered(j, i)));
      }
      // (Body, *) pairs cannot have -1.
      if (filtered(i, j) < 0 && pair_has_robot) {
        throw std::logic_error(fmt::format(
            "CollisionChecker::{}(): The filtered collision matrix can only be "
            "1 or 0 for a pair with a robot body ({}, {}), found {}.",
            func, i, j, filtered(i, j)));
      }
    }
  }
}

Eigen::MatrixXi CollisionChecker::GenerateFilteredCollisionMatrix() const {
  const int num_bodies = plant().num_bodies();
  // Initialize matrix to zero (no filtered collisions).
  Eigen::MatrixXi filtered_collisions =
      Eigen::MatrixXi::Zero(num_bodies, num_bodies);

  const auto& inspector = model().scene_graph().model_inspector();

  // Generate a mapping from body to subgraph for use in identifying welds.
  std::vector<int> body_subgraph_mapping(num_bodies, -1);

  const std::vector<std::set<BodyIndex>> subgraphs =
      plant().FindSubgraphsOfWeldedBodies();

  for (size_t subgraph_id = 0; subgraph_id < subgraphs.size(); ++subgraph_id) {
    const std::set<BodyIndex>& subgraph = subgraphs.at(subgraph_id);
    for (const BodyIndex& body_id : subgraph) {
      body_subgraph_mapping.at(body_id) = static_cast<int>(subgraph_id);
    }
  }

  // For consistency, (B, B) is always filtered.
  // Loop variables below use `int` for Eigen indexing compatibility.
  for (int i = 0; i < num_bodies; ++i) {
    filtered_collisions(i, i) = -1;

    const int body_i_subgraph_id = body_subgraph_mapping.at(i);
    // We expect FindSubgraphsOfWeldedBodies to cover all bodies, but check to
    // make sure no bodies are left with the default subgraph id.
    DRAKE_DEMAND(body_i_subgraph_id >= 0);

    const bool i_is_robot = IsPartOfRobot(BodyIndex(i));

    const RigidBody<double>& body_i = get_body(BodyIndex(i));

    const std::vector<GeometryId>& geometries_i =
        plant().GetCollisionGeometriesForBody(body_i);

    for (int j = i + 1; j < num_bodies; ++j) {
      const int body_j_subgraph_id = body_subgraph_mapping.at(j);

      const bool j_is_robot = IsPartOfRobot(BodyIndex(j));
      // (Env, env) pairs are immutably filtered (marked -1).
      if (!(i_is_robot || j_is_robot)) {
        filtered_collisions(i, j) = -1;
        filtered_collisions(j, i) = -1;
        continue;
      }

      const RigidBody<double>& body_j = get_body(BodyIndex(j));

      // Check if collisions between the geometries are already filtered.
      bool collisions_filtered = false;
      const std::vector<GeometryId>& geometries_j =
          plant().GetCollisionGeometriesForBody(body_j);
      if (geometries_i.size() > 0 && geometries_j.size() > 0) {
        collisions_filtered =
            inspector.CollisionFiltered(geometries_i.at(0), geometries_j.at(0));
        // Ensure that the collision filtering is homogeneous across all body
        // geometries.
        for (const auto& id_i : geometries_i) {
          for (const auto& id_j : geometries_j) {
            const bool current_filtered =
                inspector.CollisionFiltered(id_i, id_j);
            if (current_filtered != collisions_filtered) {
              throw std::runtime_error(fmt::format(
                  "SceneGraph's collision filters on the geometries of bodies "
                  " {} [{}] and {} [{}] are not homogeneous",
                  i, body_i.scoped_name(), j, body_j.scoped_name()));
            }
          }
        }
      }

      // While MbP already filters collisions in SceneGraph between welded
      // bodies, this is only recorded if both bodies have collision geometries
      // when this filter is applied. Since we want to handle collision
      // geometries added later, we must record if bodies are welded together.

      // If the body pair has a welded path between them, it should be filtered.
      const bool bodies_welded_together =
          body_i_subgraph_id == body_j_subgraph_id;

      // Add the filter accordingly.
      if (collisions_filtered) {
        // Filter the collision
        log()->debug(
            "Collision between body {} [{}] and body {} [{}] filtered "
            "(filtered in SceneGraph)",
            body_i.scoped_name(), i, body_j.scoped_name(), j);
        filtered_collisions(i, j) = 1;
        filtered_collisions(j, i) = 1;
      } else if (bodies_welded_together) {
        // Filter the collision
        log()->debug(
            "Collision between body {} [{}] and body {} [{}] filtered "
            "(bodies are welded together)",
            body_i.scoped_name(), i, body_j.scoped_name(), j);
        filtered_collisions(i, j) = 1;
        filtered_collisions(j, i) = 1;
      } else {
        log()->debug(
            "Collision between body {} [{}] and body {} [{}] not filtered",
            body_i.scoped_name(), i, body_j.scoped_name(), j);
      }
    }
  }
  return filtered_collisions;
}

void CollisionChecker::UpdateMaxCollisionPadding() {
  max_collision_padding_ = -std::numeric_limits<double>::infinity();
  const int N = plant().num_bodies();
  // We want to exclude the diagonal (which is always zero) so that the
  // maximum padding can be negative. We accomplish this by walking the upper
  // triangle of the padding matrix, relying on the symmetry invariant.
  // We also need to exclude environment-environment pairs.
  for (int r = 0; r < N - 1; ++r) {
    const bool r_is_robot = IsPartOfRobot(BodyIndex(r));
    for (int c = r + 1; c < N; ++c) {
      const bool c_is_robot = IsPartOfRobot(BodyIndex(c));
      if (r_is_robot || c_is_robot) {
        max_collision_padding_ =
            std::max(max_collision_padding_, collision_padding_(r, c));
      }
    }
  }
  if (!std::isfinite(max_collision_padding_)) {
    // If there are *no* robot bodies, we'd end up returning -infinity.
    max_collision_padding_ = 0;
  }
}

void CollisionChecker::ValidatePaddingMatrix(const Eigen::MatrixXd& padding,
                                             const char* func) const {
  const std::string criticism = CriticizePaddingMatrix(padding, func);
  if (!criticism.empty()) {
    throw std::logic_error(criticism);
  }
}

std::string CollisionChecker::CriticizePaddingMatrix(
    const Eigen::MatrixXd& padding, const char* func) const {
  if (padding.rows() != padding.cols()) {
    return fmt::format(
        "CollisionChecker::{}(): The padding"
        " matrix must be square. The given padding matrix is the"
        " wrong shape: {}x{}.",
        func, padding.rows(), padding.cols());
  }
  const int count = padding.rows();
  // TODO(sean.curtis): These messages have the prefix CollisionChecker.
  // Consider whether that should be replaced with NiceTypeName::Get(*this).
  // Loop variables below use `int` for Eigen indexing compatibility.
  for (int i = 0; i < count; ++i) {
    if (padding(i, i) != 0.0) {
      return fmt::format(
          "CollisionChecker::{}(): The collision padding matrix has invalid "
          "values on the diagonal ({}, {}) = {}; the values on the diagonal "
          "must always be 0.",
          func, i, i, padding(i, i));
    }

    const bool i_is_robot = IsPartOfRobot(BodyIndex(i));
    for (int j = i + 1; j < count; ++j) {
      const bool pair_has_robot = i_is_robot || IsPartOfRobot(BodyIndex(j));
      // Both are environment bodies.
      if (!pair_has_robot && padding(i, j) != 0.0) {
        return fmt::format(
            "CollisionChecker::{}(): The collision padding matrix must contain "
            "0 for pairs of environment bodies. Found {} at ({}, {}).",
            func, padding(i, j), i, j);
      }
      // Values must be finite.
      if (!std::isfinite(padding(i, j))) {
        return fmt::format(
            "CollisionChecker::{}(): The collision padding matrix must contain "
            "finite values. Found {} at ({}, {}).",
            func, padding(i, j), i, j);
      }
      // Matrix must be symmetric.
      if (padding(i, j) != padding(j, i)) {
        return fmt::format(
            "CollisionChecker::{}(): The collision padding matrix must be "
            "symmetric. Values at ({}, {}) and ({}, {}) are not equal; "
            "{} != {}.",
            func, i, j, j, i, padding(i, j), padding(j, i));
      }
    }
  }
  return {};
}

int CollisionChecker::GetNumberOfThreads(const Parallelism parallelize) const {
  const bool check_in_parallel =
      CanEvaluateInParallel() && parallelize.num_threads() > 1;
  if (check_in_parallel) {
    return std::min(num_allocated_contexts(), parallelize.num_threads());
  } else {
    return 1;
  }
}

CollisionChecker::OwnedContextKeeper::~OwnedContextKeeper() = default;

CollisionChecker::OwnedContextKeeper::OwnedContextKeeper(
    const CollisionChecker::OwnedContextKeeper& other) {
  AllocateOwnedContexts(other.prototype_context(), other.num_contexts());
}

void CollisionChecker::OwnedContextKeeper::
    PerformOperationAgainstAllOwnedContexts(
        const RobotDiagram<double>& model,
        const std::function<void(const RobotDiagram<double>&,
                                 CollisionCheckerContext*)>& operation) {
  DRAKE_DEMAND(operation != nullptr);
  DRAKE_THROW_UNLESS(allocated());
  for (auto& model_context : model_contexts_) {
    operation(model, model_context.get());
  }
  operation(model, prototype_context_.get());
}

CollisionChecker::StandaloneContextReferenceKeeper::
    ~StandaloneContextReferenceKeeper() = default;

void CollisionChecker::StandaloneContextReferenceKeeper::AddStandaloneContext(
    const std::shared_ptr<CollisionCheckerContext>& standalone_context) const {
  std::lock_guard<std::mutex> lock(standalone_contexts_mutex_);
  standalone_contexts_.push_back(
      std::weak_ptr<CollisionCheckerContext>(standalone_context));
}

void CollisionChecker::StandaloneContextReferenceKeeper::
    PerformOperationAgainstAllStandaloneContexts(
        const RobotDiagram<double>& model,
        const std::function<void(const RobotDiagram<double>&,
                                 CollisionCheckerContext*)>& operation) {
  DRAKE_DEMAND(operation != nullptr);
  std::lock_guard<std::mutex> lock(standalone_contexts_mutex_);
  for (auto standalone_context = standalone_contexts_.begin();
       standalone_context != standalone_contexts_.end();) {
    auto maybe_context = standalone_context->lock();
    if (maybe_context != nullptr) {
      operation(model, maybe_context.get());
      // Advance to next item.
      ++standalone_context;
    } else {
      // If the pointed-to context is no longer alive, remove it.
      // Note that erase() returns the iterator to the next item.
      standalone_context = standalone_contexts_.erase(standalone_context);
    }
  }
}

}  // namespace planning
}  // namespace drake

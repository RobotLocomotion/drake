#include "drake/planning/sampling_based/dev/roadmap_internal.h"

#include <utility>

using common_robotics_utilities::serialization::Deserialized;
using common_robotics_utilities::serialization::DeserializeIsometry3d;
using common_robotics_utilities::serialization::DeserializeMapLike;
using common_robotics_utilities::serialization::DeserializeString;
using common_robotics_utilities::serialization::DeserializeVector2d;
using common_robotics_utilities::serialization::DeserializeVector3d;
using common_robotics_utilities::serialization::DeserializeVectorXd;
using common_robotics_utilities::serialization::MakeDeserialized;
using common_robotics_utilities::serialization::SerializeIsometry3d;
using common_robotics_utilities::serialization::SerializeMapLike;
using common_robotics_utilities::serialization::SerializeString;
using common_robotics_utilities::serialization::SerializeVector2d;
using common_robotics_utilities::serialization::SerializeVector3d;
using common_robotics_utilities::serialization::SerializeVectorXd;

namespace drake {
namespace planning {
namespace internal {
namespace {
// Signature for templated state serialization. Signature matches serialization
// methods in common_robotics_utilities.
template <typename StateType>
uint64_t SerializeStateType(const StateType& state,
                            // NOLINTNEXTLINE(runtime/references)
                            std::vector<uint8_t>& buffer);

// Concrete implementations of state serialization for supported state types.
template <>
uint64_t SerializeStateType(const Eigen::Vector2d& state,
                            // NOLINTNEXTLINE(runtime/references)
                            std::vector<uint8_t>& buffer) {
  return SerializeVector2d(state, buffer);
}

template <>
uint64_t SerializeStateType(const Eigen::Vector3d& state,
                            // NOLINTNEXTLINE(runtime/references)
                            std::vector<uint8_t>& buffer) {
  return SerializeVector3d(state, buffer);
}

template <>
uint64_t SerializeStateType(const drake::math::RigidTransformd& state,
                            // NOLINTNEXTLINE(runtime/references)
                            std::vector<uint8_t>& buffer) {
  return SerializeIsometry3d(state.GetAsIsometry3(), buffer);
}

template <>
uint64_t SerializeStateType(const Eigen::VectorXd& state,
                            // NOLINTNEXTLINE(runtime/references)
                            std::vector<uint8_t>& buffer) {
  return SerializeVectorXd(state, buffer);
}

template <>
uint64_t SerializeStateType(const ControlPlanningState<Eigen::Vector2d>& state,
                            // NOLINTNEXTLINE(runtime/references)
                            std::vector<uint8_t>& buffer) {
  uint64_t bytes_written = SerializeVector2d(state.state(), buffer);
  bytes_written += SerializeVectorXd(state.control(), buffer);
  return bytes_written;
}

template <>
uint64_t SerializeStateType(const ControlPlanningState<Eigen::Vector3d>& state,
                            // NOLINTNEXTLINE(runtime/references)
                            std::vector<uint8_t>& buffer) {
  uint64_t bytes_written = SerializeVector3d(state.state(), buffer);
  bytes_written += SerializeVectorXd(state.control(), buffer);
  return bytes_written;
}

template <>
uint64_t SerializeStateType(
    const ControlPlanningState<drake::math::RigidTransformd>& state,
    // NOLINTNEXTLINE(runtime/references)
    std::vector<uint8_t>& buffer) {
  uint64_t bytes_written =
      SerializeIsometry3d(state.state().GetAsIsometry3(), buffer);
  bytes_written += SerializeVectorXd(state.control(), buffer);
  return bytes_written;
}

template <>
uint64_t SerializeStateType(const ControlPlanningState<Eigen::VectorXd>& state,
                            // NOLINTNEXTLINE(runtime/references)
                            std::vector<uint8_t>& buffer) {
  uint64_t bytes_written = SerializeVectorXd(state.state(), buffer);
  bytes_written += SerializeVectorXd(state.control(), buffer);
  return bytes_written;
}

// Signature for templated state deserialization. Signature matches
// deserialization methods in common_robotics_utilities.
template <typename StateType>
Deserialized<StateType> DeserializeStateType(const std::vector<uint8_t>& buffer,
                                             uint64_t starting_offset);

// Concrete implementations of state deserialization for supported state types.
template <>
Deserialized<Eigen::Vector2d> DeserializeStateType(
    const std::vector<uint8_t>& buffer, uint64_t starting_offset) {
  return DeserializeVector2d(buffer, starting_offset);
}

template <>
Deserialized<Eigen::Vector3d> DeserializeStateType(
    const std::vector<uint8_t>& buffer, uint64_t starting_offset) {
  return DeserializeVector3d(buffer, starting_offset);
}

template <>
Deserialized<drake::math::RigidTransformd> DeserializeStateType(
    const std::vector<uint8_t>& buffer, uint64_t starting_offset) {
  auto deserialized_isometry3d = DeserializeIsometry3d(buffer, starting_offset);
  return MakeDeserialized(
      drake::math::RigidTransformd(deserialized_isometry3d.Value()),
      deserialized_isometry3d.BytesRead());
}

template <>
Deserialized<Eigen::VectorXd> DeserializeStateType(
    const std::vector<uint8_t>& buffer, uint64_t starting_offset) {
  return DeserializeVectorXd(buffer, starting_offset);
}

template <>
Deserialized<ControlPlanningState<Eigen::Vector2d>> DeserializeStateType(
    const std::vector<uint8_t>& buffer, uint64_t starting_offset) {
  uint64_t current_offset = starting_offset;
  auto deserialized_state = DeserializeVector2d(buffer, current_offset);
  current_offset += deserialized_state.BytesRead();
  auto deserialized_control = DeserializeVectorXd(buffer, current_offset);
  current_offset += deserialized_control.BytesRead();
  const uint64_t bytes_read = current_offset - starting_offset;
  return MakeDeserialized(
      ControlPlanningState<Eigen::Vector2d>(deserialized_state.Value(),
                                            deserialized_control.Value()),
      bytes_read);
}

template <>
Deserialized<ControlPlanningState<Eigen::Vector3d>> DeserializeStateType(
    const std::vector<uint8_t>& buffer, uint64_t starting_offset) {
  uint64_t current_offset = starting_offset;
  auto deserialized_state = DeserializeVector3d(buffer, current_offset);
  current_offset += deserialized_state.BytesRead();
  auto deserialized_control = DeserializeVectorXd(buffer, current_offset);
  current_offset += deserialized_control.BytesRead();
  const uint64_t bytes_read = current_offset - starting_offset;
  return MakeDeserialized(
      ControlPlanningState<Eigen::Vector3d>(deserialized_state.Value(),
                                            deserialized_control.Value()),
      bytes_read);
}

template <>
Deserialized<ControlPlanningState<drake::math::RigidTransformd>>
DeserializeStateType(const std::vector<uint8_t>& buffer,
                     uint64_t starting_offset) {
  uint64_t current_offset = starting_offset;
  auto deserialized_state = DeserializeIsometry3d(buffer, current_offset);
  current_offset += deserialized_state.BytesRead();
  auto deserialized_control = DeserializeVectorXd(buffer, current_offset);
  current_offset += deserialized_control.BytesRead();
  const uint64_t bytes_read = current_offset - starting_offset;
  return MakeDeserialized(
      ControlPlanningState<drake::math::RigidTransformd>(
          drake::math::RigidTransformd(deserialized_state.Value()),
          deserialized_control.Value()),
      bytes_read);
}

template <>
Deserialized<ControlPlanningState<Eigen::VectorXd>> DeserializeStateType(
    const std::vector<uint8_t>& buffer, uint64_t starting_offset) {
  uint64_t current_offset = starting_offset;
  auto deserialized_state = DeserializeVectorXd(buffer, current_offset);
  current_offset += deserialized_state.BytesRead();
  auto deserialized_control = DeserializeVectorXd(buffer, current_offset);
  current_offset += deserialized_control.BytesRead();
  const uint64_t bytes_read = current_offset - starting_offset;
  return MakeDeserialized(
      ControlPlanningState<Eigen::VectorXd>(deserialized_state.Value(),
                                            deserialized_control.Value()),
      bytes_read);
}
}  // namespace

template <typename StateType>
uint64_t RoadmapSerializer<StateType>::SerializeRoadmap(
    const Roadmap<StateType>& roadmap,
    // NOLINTNEXTLINE(runtime/references)
    std::vector<uint8_t>& buffer) {
  return RoadmapGraph<StateType>::Serialize(
      GetRoadmapInternalGraph(roadmap), buffer, SerializeStateType<StateType>);
}

template <typename StateType>
DeserializedRoadmap<StateType> RoadmapSerializer<StateType>::DeserializeRoadmap(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset) {
  auto deser_roadmap_graph = RoadmapGraph<StateType>::Deserialize(
      buffer, starting_offset, DeserializeStateType<StateType>);
  Roadmap<StateType> roadmap;
  auto& roadmap_graph = GetMutableRoadmapInternalGraph(roadmap);
  roadmap_graph = std::move(deser_roadmap_graph.MutableValue());
  return MakeDeserialized(roadmap, deser_roadmap_graph.BytesRead());
}

template <typename StateType>
uint64_t RoadmapSerializer<StateType>::SerializeNamedRoadmaps(
    const std::map<std::string, Roadmap<StateType>>& named_roadmaps,
    // NOLINTNEXTLINE(runtime/references)
    std::vector<uint8_t>& buffer) {
  return SerializeMapLike<std::string, Roadmap<StateType>>(
      named_roadmaps, buffer, SerializeString<char>, SerializeRoadmap);
}

template <typename StateType>
DeserializedNamedRoadmaps<StateType>
RoadmapSerializer<StateType>::DeserializeNamedRoadmaps(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset) {
  return DeserializeMapLike<std::string, Roadmap<StateType>>(
      buffer, starting_offset, DeserializeString<char>, DeserializeRoadmap);
}
}  // namespace internal
}  // namespace planning
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::internal::RoadmapSerializer)

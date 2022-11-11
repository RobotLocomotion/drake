#pragma once

// This file is derived from the python prototype at
// https://github.com/EricCousineau-TRI/repro/blob/c44615ee725b9c5498cd5b1d706ccfec6c3bd203/drake_stuff/multibody_plant_prototypes/multibody_plant_subgraph.py.
// Only a subset of the functionality that is necessary for handling custom
// parsed models included via SDFormat's merge-include has been ported.

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/scoped_names.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/universal_joint.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace multibody {
namespace internal {

std::vector<ModelInstanceIndex> GetModelInstances(
    const MultibodyPlant<double>& plant);

std::vector<const Body<double>*> GetBodies(
    const MultibodyPlant<double>& plant,
    std::optional<std::vector<ModelInstanceIndex>> model_instances =
        std::nullopt);

std::vector<const Joint<double>*> GetJoints(
    const MultibodyPlant<double>& plant,
    std::optional<std::vector<ModelInstanceIndex>> model_instances =
        std::nullopt);

Eigen::VectorXd GetJointPositions(const MultibodyPlant<double>& plant,
                                  const systems::Context<double>& context,
                                  const Joint<double>& joint);

void SetJointPositions(MultibodyPlant<double>* plant,
                       systems::Context<double>* context,
                       const Joint<double>& joint,
                       const Eigen::Ref<const VectorX<double>>& qj);

Eigen::VectorXd GetJointVelocities(const MultibodyPlant<double>& plant,
                                   const systems::Context<double>& context,
                                   const Joint<double>& joint);

void SetJointVelocities(MultibodyPlant<double>* plant,
                        systems::Context<double>* context,
                        const Joint<double>& joint,
                        const Eigen::Ref<const VectorX<double>>& vj);

struct CompareElements {
  template <typename T>
  bool operator()(const T* a, const T* b) const {
    return a->index() < b->index();
  }
};

template <typename T>
bool IsDisjoint(const T& a, const T& b) {
  T intersection;
  std::set_intersection(a.begin(), a.end(), b.begin(), b.end(),
                        std::inserter(intersection, intersection.begin()));
  return intersection.empty();
}

template <typename T>
void ExclusiveSetUpdate(T* dest, const T& src) {
  DRAKE_THROW_UNLESS(IsDisjoint(*dest, src));
  dest->insert(src.begin(), src.end());
}

template <typename T>
using SortedSet = std::set<T, CompareElements>;

class MultibodyPlantElementsMap;
class MultibodyPlantSubgraph;

class MultibodyPlantElements {
 public:
  MultibodyPlantElements() = default;
  explicit MultibodyPlantElements(
      const MultibodyPlant<double>* plant,
      const geometry::SceneGraph<double>* scene_graph = nullptr)
      : plant_(plant), scene_graph_(scene_graph) {}

  static MultibodyPlantElements FromPlant(
      const MultibodyPlant<double>* plant,
      const geometry::SceneGraph<double>* scene_graph = nullptr);

  void PrintBodies(std::ostream& out) const {
    for (const auto& body : bodies()) {
      out << "Body: " << body << " " << body->name()
          << " model: " << body->model_instance() << std::endl;
    }
  }
  void PrintFrames(std::ostream& out) const {
    for (const auto& frame : frames()) {
      out << "Frame: " << frame << " " << frame->name()
          << " model: " << frame->model_instance()
          << " attached_to: " << frame->body().name() << std::endl;
    }
  }
  void PrintJoints(std::ostream& out) const {
    for (const auto& joint : joints()) {
      out << "Joint: " << joint << " " << joint->name()
          << " model: " << joint->model_instance()
          << " child: " << joint->child_body().name()
          << " parent: " << joint->parent_body().name() << std::endl;
    }
  }
  void PrintModels(std::ostream& out) const {
    for (const auto& model : model_instances()) {
      out << "Model: " << model << " " << plant()->GetModelInstanceName(model)
          << std::endl;
    }
  }
  std::string GeometryToString(geometry::GeometryId id) const {
    const geometry::SceneGraphInspector<double>* inspector{nullptr};
    if (scene_graph() != nullptr) {
      inspector = &scene_graph()->model_inspector();
    }
    std::stringstream out;
    if (inspector != nullptr) {
      out << "name: " << inspector->GetName(id) << " ";
    }
    out << "id:" << id;
    return out.str();
  }

  void PrintGeometries(std::ostream& out) const {
    for (const auto& id : geometry_ids()) {
      out << "Geometry: " << GeometryToString(id) << "\n";
    }
  }

  void PrintCollisionFilterPairs(std::ostream& out) const {
    out << "Collision filter pairs" << std::endl;
    for (const auto& [id_a, id_b] : collision_filter_pairs()) {
      out << fmt::format("({}, {})\n", GeometryToString(id_a),
                         GeometryToString(id_b));
    }
  }

  void PrintAll(std::ostream& out) const {
    PrintModels(out);
    PrintBodies(out);
    PrintFrames(out);
    PrintJoints(out);
    PrintGeometries(out);
    PrintCollisionFilterPairs(out);
  }

  const MultibodyPlant<double>* plant() const { return plant_; }
  const geometry::SceneGraph<double>* scene_graph() const {
    return scene_graph_;
  }

  void ResetSceneGraph() {
    scene_graph_ = nullptr;
    geometry_ids_.clear();
    collision_filter_pairs_.clear();
  }

  const std::set<ModelInstanceIndex>& model_instances() const {
    return model_instances_;
  }
  std::set<ModelInstanceIndex>& model_instances() { return model_instances_; }

  const std::set<geometry::GeometryId>& geometry_ids() const {
    return geometry_ids_;
  }

  std::set<geometry::GeometryId>& geometry_ids() { return geometry_ids_; }

  const SortedSet<const Body<double>*>& bodies() const { return bodies_; }

  SortedSet<const Body<double>*>& bodies() { return bodies_; }

  const SortedSet<const Frame<double>*>& frames() const { return frames_; }

  SortedSet<const Frame<double>*>& frames() { return frames_; }

  const SortedSet<const Joint<double>*>& joints() const { return joints_; }

  SortedSet<const Joint<double>*>& joints() { return joints_; }

  const SortedSet<const JointActuator<double>*>& joint_actuators() const {
    return joint_actuators_;
  }

  SortedSet<const JointActuator<double>*>& joint_actuators() {
    return joint_actuators_;
  }

  const std::set<std::pair<geometry::GeometryId, geometry::GeometryId>>&
  collision_filter_pairs() const {
    return collision_filter_pairs_;
  }

  MultibodyPlantElements operator+=(const MultibodyPlantElements& other) {
    Check(other);
    ExclusiveSetUpdate(&model_instances_, other.model_instances_);
    ExclusiveSetUpdate(&bodies_, other.bodies_);
    ExclusiveSetUpdate(&frames_, other.frames_);
    ExclusiveSetUpdate(&joints_, other.joints_);
    ExclusiveSetUpdate(&joint_actuators_, other.joint_actuators_);
    ExclusiveSetUpdate(&geometry_ids_, other.geometry_ids_);
    ExclusiveSetUpdate(&collision_filter_pairs_, other.collision_filter_pairs_);
    return *this;
  }

  friend bool operator==(const MultibodyPlantElements& a,
                         const MultibodyPlantElements& b) {
    return a.model_instances() == b.model_instances() &&
           a.bodies() == b.bodies() && a.frames() == b.frames() &&
           a.joints() == b.joints() &&
           a.joint_actuators() == b.joint_actuators() &&
           a.geometry_ids() == b.geometry_ids() &&
           a.collision_filter_pairs() == b.collision_filter_pairs();
  }

  friend bool operator!=(const MultibodyPlantElements& a,
                         const MultibodyPlantElements& b) {
    return !(a == b);
  }

 private:
  static MultibodyPlantElements GetElementsFromBodies(
      const MultibodyPlant<double>* plant,
      const std::vector<const Body<double>*>& bodies,
      const geometry::SceneGraph<double>* scene_graph,
      std::optional<std::vector<ModelInstanceIndex>> model_instances =
          std::nullopt);

  void Check(const MultibodyPlantElements& other) {
    DRAKE_THROW_UNLESS(plant_ == other.plant_);
    DRAKE_THROW_UNLESS(scene_graph_ == other.scene_graph_);
  }

  const MultibodyPlant<double>* plant_{nullptr};
  const geometry::SceneGraph<double>* scene_graph_{nullptr};

  std::set<ModelInstanceIndex> model_instances_;
  SortedSet<const Body<double>*> bodies_;
  SortedSet<const Frame<double>*> frames_;
  SortedSet<const Joint<double>*> joints_;
  SortedSet<const JointActuator<double>*> joint_actuators_;
  std::set<geometry::GeometryId> geometry_ids_;
  std::set<std::pair<geometry::GeometryId, geometry::GeometryId>>
      collision_filter_pairs_;
};

using FrameNameRemapFunction = std::function<std::string(const Frame<double>&)>;

// Forward declaration
class MultibodySubgraphElementAccessor;

class MultibodyPlantElementsMap {
 public:
  MultibodyPlantElementsMap(const MultibodyPlant<double>* src,
                            MultibodyPlant<double>* dest,
                            const geometry::SceneGraph<double>* scene_graph_src)
      : plant_src_(src),
        plant_dest_(dest),
        scene_graph_src_(scene_graph_src),
        builtins_src_(MakeEmptyElementsSrc()) {}

  MultibodyPlantElements MakeEmptyElementsSrc() {
    return MultibodyPlantElements(plant_src_, scene_graph_src_);
  }

  void RegisterWorldBodyAndFrame() {
    bodies_.insert({&plant_src_->world_body(), &plant_dest_->world_body()});
    builtins_src_.bodies().insert(&plant_src_->world_body());

    frames_.insert({&plant_src_->world_frame(), &plant_dest_->world_frame()});
    builtins_src_.frames().insert(&plant_src_->world_frame());
  }

  void RegisterModelInstance(ModelInstanceIndex src, ModelInstanceIndex dest) {
    model_instances_.insert({src, dest});
  }

  void CopyBody(const Body<double>* src,
                const MultibodySubgraphElementAccessor& handle);

  void CopyFrame(const Frame<double>* src,
                 const MultibodySubgraphElementAccessor& handle);

  void CopyJoint(const Joint<double>* src,
                 const MultibodySubgraphElementAccessor& handle);

  void CopyJointActuator(const JointActuator<double>* src,
                         const MultibodySubgraphElementAccessor& handle);

  void CopyGeometryById(const geometry::GeometryId& geometry_id_src);

  void CopyCollisionFilterPair(
      const std::pair<geometry::GeometryId, geometry::GeometryId>& filter_pair);

  // Copies the (physical) state from context_src to context_dest.
  void CopyState(const systems::Context<double>& context_src,
                 systems::Context<double>* context_dest);

  const std::map<ModelInstanceIndex, ModelInstanceIndex>& model_instances()
      const {
    return model_instances_;
  }

  std::map<ModelInstanceIndex, ModelInstanceIndex>& model_instances() {
    return model_instances_;
  }


  const std::map<const Body<double>*, const Body<double>*>& bodies() const {
    return bodies_;
  }

  std::map<const Body<double>*, const Body<double>*>& bodies() {
    return bodies_;
  }

  const std::map<const Frame<double>*, const Frame<double>*>& frames() const {
    return frames_;
  }

  std::map<const Frame<double>*, const Frame<double>*>& frames() {
    return frames_;
  }

  const std::map<const Joint<double>*, const Joint<double>*>& joints() const {
    return joints_;
  }

  std::map<const Joint<double>*, const Joint<double>*>& joints() {
    return joints_;
  }

  const std::map<const JointActuator<double>*, const JointActuator<double>*>&
  joint_actuators() const {
    return joint_actuators_;
  }

  std::map<const JointActuator<double>*, const JointActuator<double>*>&
  joint_actuators() {
    return joint_actuators_;
  }

  const std::map<geometry::GeometryId, geometry::GeometryId>& geometry_ids()
      const {
    return geometry_ids_;
  }

  std::map<geometry::GeometryId, geometry::GeometryId>& geometry_ids() {
    return geometry_ids_;
  }

  MultibodyPlant<double>* plant_destination() {
    return plant_dest_;
  }

  friend bool operator==(const MultibodyPlantElementsMap& a,
                         const MultibodyPlantElementsMap& b) {
    return a.model_instances() == b.model_instances() &&
           a.bodies() == b.bodies() && a.frames() == b.frames() &&
           a.joints() == b.joints() &&
           a.joint_actuators() == b.joint_actuators() &&
           a.geometry_ids() == b.geometry_ids();
  }
  friend bool operator!=(const MultibodyPlantElementsMap& a,
                         const MultibodyPlantElementsMap& b) {
    return !(a == b);
  }

 private:
  const MultibodyPlant<double>* plant_src_;
  MultibodyPlant<double>* plant_dest_;
  const geometry::SceneGraph<double>* scene_graph_src_;
  MultibodyPlantElements builtins_src_;
  std::map<ModelInstanceIndex, ModelInstanceIndex> model_instances_;
  std::map<const Body<double>*, const Body<double>*> bodies_;
  std::map<const Frame<double>*, const Frame<double>*> frames_;
  std::map<const Joint<double>*, const Joint<double>*> joints_;
  std::map<const JointActuator<double>*, const JointActuator<double>*>
      joint_actuators_;
  std::map<geometry::GeometryId, geometry::GeometryId> geometry_ids_;
};

// Ensures that current elements / topology satisfy subgraph invariants.
// @param elem
//   The MultibodyPlantElements object to be checked.
// @throws std::exception if any of the invariant checks fails.
void CheckSubgraphInvariants(const MultibodyPlantElements& elem);

ModelInstanceIndex GetOrCreateModelInstanceByName(
    MultibodyPlant<double>* plant, const std::string& model_name);

std::string FrameNameRenameSameName(const Frame<double>& frame);

class MultibodyPlantSubgraph {
 public:
  MultibodyPlantSubgraph() = default;
  explicit MultibodyPlantSubgraph(MultibodyPlantElements elem)
      : elem_src_(std::move(elem)) {
    CheckSubgraphInvariants(elem_src_);
  }

  using RemapFunction = std::function<ModelInstanceIndex(ModelInstanceIndex)>;

  MultibodyPlantElementsMap AddTo(
      MultibodyPlant<double>* plant_dest,
      std::optional<RemapFunction> model_instance_remap = std::nullopt,
      std::optional<FrameNameRemapFunction> frame_name_remap = std::nullopt) const;

  MultibodyPlantElementsMap AddTo(
      MultibodyPlant<double>* plant_dest,
      const std::string &model_instance_remap,
      FrameNameRemapFunction frame_name_remap = FrameNameRenameSameName) const;

  const MultibodyPlantElements& elements_src() const { return elem_src_; }

  MultibodyPlantElements MakeEmptyElements();
  MultibodyPlantElements RemoveModelInstance(ModelInstanceIndex model_instance);
  MultibodyPlantElements RemoveBody(const Body<double>* body,
                                    bool include_welded_bodies = false);
  MultibodyPlantElements RemoveFrame(const Frame<double>* frame);
  MultibodyPlantElements RemoveJoint(const Joint<double>* joint);
  MultibodyPlantElements RemoveJointActuator(
      const JointActuator<double>* joint_actuator);
  MultibodyPlantElements RemoveGeometryId(geometry::GeometryId geometry_id);

 private:
  MultibodyPlantElements elem_src_;
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake

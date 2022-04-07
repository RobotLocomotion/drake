#pragma once

// This file is derived from the python prototype at
// https://github.com/EricCousineau-TRI/repro/blob/c44615ee725b9c5498cd5b1d706ccfec6c3bd203/drake_stuff/multibody_plant_prototypes/multibody_plant_subgraph.py.
// Only a subset of the functionality that is necessary for handling custom
// parsed models included via SDFormat's merge-include has been ported.

#include <algorithm>
#include <map>
#include <memory>
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

  void PrintAll(std::ostream& out) const {
    PrintModels(out);
    PrintBodies(out);
    PrintFrames(out);
    PrintJoints(out);
  }

  const MultibodyPlant<double>* plant() const { return plant_; }
  const geometry::SceneGraph<double>* scene_graph() const {
    return scene_graph_;
  }

  const std::set<const Body<double>*>& bodies() const { return bodies_; }

  std::set<const Body<double>*>& bodies() { return bodies_; }

  const std::set<const Frame<double>*>& frames() const { return frames_; }

  std::set<const Frame<double>*>& frames() { return frames_; }

  const std::set<const Joint<double>*>& joints() const { return joints_; }

  std::set<const Joint<double>*>& joints() { return joints_; }

  const std::set<const JointActuator<double>*>& joint_actuators() const {
    return joint_actuators_;
  }

  std::set<const JointActuator<double>*>& joint_actuators() {
    return joint_actuators_;
  }

  const std::set<ModelInstanceIndex>& model_instances() const {
    return model_instances_;
  }
  std::set<ModelInstanceIndex>& model_instances() { return model_instances_; }

  std::set<geometry::GeometryId> geometry_ids() const { return geometry_ids_; }

 private:
  static MultibodyPlantElements GetElementsFromBodies(
      const MultibodyPlant<double>* plant,
      const std::vector<const Body<double>*>& bodies,
      const geometry::SceneGraph<double>* scene_graph);

  const MultibodyPlant<double>* plant_{nullptr};
  const geometry::SceneGraph<double>* scene_graph_{nullptr};

  std::set<ModelInstanceIndex> model_instances_;
  std::set<const Body<double>*> bodies_;
  std::set<const Frame<double>*> frames_;
  std::set<const Joint<double>*> joints_;
  std::set<const JointActuator<double>*> joint_actuators_;
  std::set<geometry::GeometryId> geometry_ids_;
};

using FrameNameRemapFunction = std::function<std::string(
    const MultibodyPlant<double>& plant_src, const Frame<double>&)>;

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

  void CopyBody(const Body<double>* src);

  void CopyFrame(const Frame<double>* src,
                 FrameNameRemapFunction frame_name_remap);

  void CopyJoint(const Joint<double>* src);

  void CopyJointActuator(const JointActuator<double>* src);

  void CopyGeometryById(const geometry::GeometryId& geometry_id_src);

  const std::map<ModelInstanceIndex, ModelInstanceIndex>& model_instances()
      const {
    return model_instances_;
  }

 private:
  const MultibodyPlant<double>* plant_src_;
  MultibodyPlant<double>* plant_dest_;
  const geometry::SceneGraph<double>* scene_graph_src_;
  MultibodyPlantElements builtins_src_;
  std::map<const Body<double>*, const Body<double>*> bodies_;
  std::map<const Frame<double>*, const Frame<double>*> frames_;
  std::map<const Joint<double>*, const Joint<double>*> joints_;
  std::map<const JointActuator<double>*, const JointActuator<double>*>
      joint_actuators_;
  std::map<ModelInstanceIndex, ModelInstanceIndex> model_instances_;
  std::map<geometry::GeometryId, geometry::GeometryId> geometry_ids_;
};

// Ensures that current elements / topology satisifes subgraph invariants.
// @param elem
//   The MultibodyPlantElements object to be checked.
// @throws std::exception if any of the invariant checks fails.
void CheckSubgraphInvariants(const MultibodyPlantElements& elem);

ModelInstanceIndex GetOrCreateModelInstanceByName(
    MultibodyPlant<double>* plant, const std::string& model_name);

ModelInstanceIndex ModelInstanceRemapSameName(
    const MultibodyPlant<double>& plant_src,
    ModelInstanceIndex model_instance_src, MultibodyPlant<double>* plant_dest);

std::string FrameNameRenameSameName(const MultibodyPlant<double>&,
                                    const Frame<double>& frame);

class MultibodyPlantSubgraph {
 public:
  MultibodyPlantSubgraph() = default;
  explicit MultibodyPlantSubgraph(MultibodyPlantElements elem)
      : elem_src_(std::move(elem)) {
    CheckSubgraphInvariants(elem_src_);
  }

  using RemapFunction = std::function<ModelInstanceIndex(
      const MultibodyPlant<double>&, ModelInstanceIndex,
      MultibodyPlant<double>*)>;

  MultibodyPlantElementsMap AddTo(
      MultibodyPlant<double>* plant_dest,
      RemapFunction model_instance_remap = ModelInstanceRemapSameName,
      FrameNameRemapFunction frame_name_remap = FrameNameRenameSameName) const;

  const MultibodyPlantElements& elements() const { return elem_src_; }

 private:
  MultibodyPlantElements elem_src_;
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake

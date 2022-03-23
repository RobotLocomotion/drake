#pragma once

// This file is derived from the python prototype at
// https://github.com/EricCousineau-TRI/repro/blob/master/drake_stuff/multibody_plant_prototypes/multibody_plant_subgraph.py.
// Only a subset of the functionality that is necessary for handling custom
// parsed models included via SDFormat's merge-include has been ported.

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/universal_joint.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
std::vector<T> IndicesToVector(int num_items) {
  std::vector<T> items(num_items);
  std::iota(items.begin(), items.end(), T(0));
  return items;
}

template <typename T, typename GetFuncT>
std::vector<const T*> GetPlantAggregate(int num_items, GetFuncT get_func) {
  using IndexType = decltype(std::declval<T>().index());
  std::vector<const T*> items(num_items);
  for (IndexType index(0); index < num_items; ++index) {
    items[index] = get_func(index);
  }
  return items;
}

std::vector<const Body<double>*> GetBodies(
    const MultibodyPlant<double>& plant) {
  return GetPlantAggregate<Body<double>>(
      plant.num_bodies(), [&](auto i) { return &plant.get_body(i); });
}

std::vector<const Frame<double>*> GetFrames(
    const MultibodyPlant<double>& plant) {
  return GetPlantAggregate<Frame<double>>(
      plant.num_frames(), [&](auto i) { return &plant.get_frame(i); });
}

std::vector<const Frame<double>*> GetFramesAttachedTo(
    const MultibodyPlant<double>& plant,
    const std::set<const Body<double>*>& bodies) {
  auto frames = GetFrames(plant);
  auto new_end = std::remove_if(frames.begin(), frames.end(), [&](auto frame) {
    return bodies.find(&frame->body()) == bodies.end();
  });
  frames.erase(new_end, frames.end());
  return frames;
}

std::vector<const Joint<double>*> GetJoints(
    const MultibodyPlant<double>& plant) {
  return GetPlantAggregate<Joint<double>>(
      plant.num_joints(), [&](auto i) { return &plant.get_joint(i); });
}
std::vector<const JointActuator<double>*> GetJointActuators(
    const MultibodyPlant<double>& plant) {
  return GetPlantAggregate<JointActuator<double>>(
      plant.num_actuators(),
      [&](auto i) { return &plant.get_joint_actuator(i); });
}

bool IsJointSolelyConnectedTo(const Joint<double>* joint,
                              const std::set<const Body<double>*>& bodies) {
  auto parent = bodies.find(&joint->parent_body());
  auto child = bodies.find(&joint->child_body());
  return (parent != bodies.end()) && (child != bodies.end());
}

auto GetJointsSolelyConnectedTo(const MultibodyPlant<double>& plant,
                                const std::set<const Body<double>*>& bodies) {
  auto joints = GetJoints(plant);
  auto new_end = std::remove_if(joints.begin(), joints.end(), [&](auto joint) {
    return !IsJointSolelyConnectedTo(joint, bodies);
  });
  joints.erase(new_end, joints.end());
  return joints;
}
auto GetJointActuatorsAffectingJoints(
    const MultibodyPlant<double>& plant,
    const std::set<const Joint<double>*>& joints) {
  auto joint_actuators = GetJointActuators(plant);
  auto new_end = std::remove_if(
      joint_actuators.begin(), joint_actuators.end(), [&](auto joint_actuator) {
        return joints.find(&joint_actuator->joint()) == joints.end();
      });
  joint_actuators.erase(new_end, joint_actuators.end());
  return joint_actuators;
}

class MultibodyPlantElementsMap;
class MultibodyPlantSubgraph;

class MultibodyPlantElements {
 public:
  MultibodyPlantElements() = default;
  explicit MultibodyPlantElements(const MultibodyPlant<double>* plant)
      : plant_(plant) {}

  static MultibodyPlantElements FromPlant(const MultibodyPlant<double>* plant) {
    auto model_instances =
        IndicesToVector<ModelInstanceIndex>(plant->num_model_instances());

    std::vector<const Body<double>*> bodies;

    for (auto mi : model_instances) {
      auto tmp = plant->GetBodyIndices(mi);
      bodies.reserve(bodies.size() + tmp.size());
      std::transform(tmp.begin(), tmp.end(), std::back_inserter(bodies),
                     [&](auto bi) { return &plant->get_body(bi); });
    }

    return GetElementsFromBodies(plant, bodies);
  }

  void PrintBodies() const {
    for (const auto& body : bodies()) {
      std::cout << "Body: " << body << " " << body->name()
                << " model: " << body->model_instance() << std::endl;
    }
  }
  void PrintFrames() const {
    for (const auto& frame : frames()) {
      std::cout << "Frame: " << frame << " " << frame->name()
                << " model: " << frame->model_instance()
                << " attached_to: " << frame->body().name() << std::endl;
    }
  }
  void PrintJoints() const {
    for (const auto& joint : joints()) {
      std::cout << "Joint: " << joint << " " << joint->name()
                << " model: " << joint->model_instance()
                << " child: " << joint->child_body().name()
                << " parent: " << joint->parent_body().name() << std::endl;
    }
  }
  void PrintModels() const {
    for (const auto& model : model_instances()) {
      std::cout << "Model: " << model << " "
                << plant().GetModelInstanceName(model) << std::endl;
    }
  }

  void PrintAll() const {
    PrintModels();
    PrintBodies();
    PrintFrames();
    PrintJoints();
  }

  const MultibodyPlant<double>& plant() const { return *plant_; }

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

 private:
  static MultibodyPlantElements GetElementsFromBodies(
      const MultibodyPlant<double>* plant,
      const std::vector<const Body<double>*>& bodies) {
    MultibodyPlantElements elem(plant);
    elem.bodies_.insert(bodies.begin(), bodies.end());

    for (const auto& body : elem.bodies_) {
      elem.model_instances_.insert(body->model_instance());
    }

    auto joints = GetJointsSolelyConnectedTo(*plant, elem.bodies_);
    elem.joints_.insert(joints.begin(), joints.end());

    auto joint_actuators =
        GetJointActuatorsAffectingJoints(*plant, elem.joints_);
    elem.joint_actuators_.insert(joint_actuators.begin(),
                                 joint_actuators.end());

    auto frames = GetFramesAttachedTo(*plant, elem.bodies_);
    elem.frames_.insert(frames.begin(), frames.end());

    for (const auto& frame : elem.frames_) {
      elem.model_instances_.insert(frame->model_instance());
    }
    return elem;
  }

  const MultibodyPlant<double>* plant_{nullptr};

  std::set<const Body<double>*> bodies_;
  std::set<const Frame<double>*> frames_;
  std::set<const Joint<double>*> joints_;
  std::set<const JointActuator<double>*> joint_actuators_;
  std::set<ModelInstanceIndex> model_instances_;
};

using FrameNameRemapFuncT = std::function<std::string(
    const MultibodyPlant<double>& plant_src, const Frame<double>&)>;

class MultibodyPlantElementsMap {
 public:
  MultibodyPlantElementsMap(const MultibodyPlant<double>* src,
                            MultibodyPlant<double>* dest)
      : plant_src_(src),
        plant_dest_(dest),
        builtins_src_(MakeEmptyElementsSrc()) {}

  MultibodyPlantElements MakeEmptyElementsSrc() {
    return MultibodyPlantElements(plant_src_);
  }
  MultibodyPlantElements MakeEmptyElementsDest() {
    return MultibodyPlantElements(plant_dest_);
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

  void CopyBody(const Body<double>* src) {
    if (builtins_src_.bodies().find(src) != builtins_src_.bodies().end()) {
      return;
    }
    auto body_src = dynamic_cast<const RigidBody<double>*>(src);
    DRAKE_DEMAND(body_src != nullptr);

    const ModelInstanceIndex model_instance_src = body_src->model_instance();
    const ModelInstanceIndex model_instance_dest =
        model_instances_[model_instance_src];
    const Body<double>* body_dest =
        &plant_dest_->AddRigidBody(body_src->name(), model_instance_dest,
                                   body_src->default_spatial_inertia());
    bodies_.insert({body_src, body_dest});

    // Set default state.
    const auto X_WB = plant_src_->GetDefaultFreeBodyPose(*body_src);
    plant_dest_->SetDefaultFreeBodyPose(*body_dest, X_WB);
    // Register body frame as a builtin.
    const Frame<double>* frame_src = &body_src->body_frame();
    const Frame<double>* frame_dest = &body_dest->body_frame();
    builtins_src_.frames().insert(frame_src);
    frames_.insert({frame_src, frame_dest});
  }

  void CopyFrame(const Frame<double>* src,
                 FrameNameRemapFuncT frame_name_remap) {
    if (builtins_src_.frames().find(src) != builtins_src_.frames().end()) {
      return;
    }
    // BodyFrame's are handled by `CopyBody`, and are ignored by this method.
    DRAKE_DEMAND(dynamic_cast<const BodyFrame<double>*>(src) == nullptr);

    const Frame<double>* parent_frame_src = &src->body().body_frame();
    const Frame<double>* parent_frame_dest = frames_[parent_frame_src];
    const ModelInstanceIndex model_instance_src = src->model_instance();
    const ModelInstanceIndex model_instance_dest =
        model_instances_[model_instance_src];
    DRAKE_DEMAND(model_instance_dest.is_valid());

    const Frame<double>* frame_dest =
        &plant_dest_->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
            frame_name_remap(*plant_src_, *src), *parent_frame_dest,
            src->GetFixedPoseInBodyFrame(), model_instance_dest));

    frames_.insert({src, frame_dest});
  }

  void CopyJoint(const Joint<double>* src) {
    const Frame<double>* frame_on_parent_dest =
        frames_[&src->frame_on_parent()];
    const Frame<double>* frame_on_child_dest = frames_[&src->frame_on_child()];

    std::unique_ptr<Joint<double>> joint;
    // TODO(azeey) Not sure if this matches the python prototype.
    if (auto ball_joint_src = dynamic_cast<const BallRpyJoint<double>*>(src);
        ball_joint_src != nullptr) {
      joint = std::make_unique<BallRpyJoint<double>>(
          ball_joint_src->name(), *frame_on_parent_dest, *frame_on_child_dest,
          ball_joint_src->damping());
    } else if (auto prismatic_joint_src =
                   dynamic_cast<const PrismaticJoint<double>*>(src);
               prismatic_joint_src != nullptr) {
      joint = std::make_unique<PrismaticJoint<double>>(
          prismatic_joint_src->name(), *frame_on_parent_dest,
          *frame_on_child_dest, prismatic_joint_src->translation_axis(),
          prismatic_joint_src->damping());
    } else if (auto revolute_joint_src =
                   dynamic_cast<const RevoluteJoint<double>*>(src);
               revolute_joint_src != nullptr) {
      joint = std::make_unique<RevoluteJoint<double>>(
          revolute_joint_src->name(), *frame_on_parent_dest,
          *frame_on_child_dest, revolute_joint_src->revolute_axis(),
          revolute_joint_src->damping());
    } else if (auto universal_joint_src =
                   dynamic_cast<const UniversalJoint<double>*>(src);
               universal_joint_src != nullptr) {
      joint = std::make_unique<UniversalJoint<double>>(
          universal_joint_src->name(), *frame_on_parent_dest,
          *frame_on_child_dest, universal_joint_src->damping());
    } else if (auto weld_joint_src =
                   dynamic_cast<const WeldJoint<double>*>(src);
               weld_joint_src != nullptr) {
      joint = std::make_unique<WeldJoint<double>>(
          weld_joint_src->name(), *frame_on_parent_dest, *frame_on_child_dest,
          weld_joint_src->X_PC());
    } else {
      throw std::runtime_error(
          fmt::format("Cannot clone joint type: {}", joint->type_name()));
    }

    joint->set_position_limits(src->position_lower_limits(),
                               src->position_upper_limits());
    joint->set_velocity_limits(src->velocity_lower_limits(),
                               src->velocity_upper_limits());
    joint->set_acceleration_limits(src->acceleration_lower_limits(),
                                   src->acceleration_upper_limits());
    joint->set_default_positions(src->default_positions());
    const Joint<double>* joint_dest = &plant_dest_->AddJoint(std::move(joint));
    joints_.insert({src, joint_dest});
  }

  void CopyJointActuator(const JointActuator<double>* src) {
    const Joint<double>* joint_src = &src->joint();
    const Joint<double>* joint_dest = joints_[joint_src];
    const JointActuator<double>* joint_actuator_dest =
        &plant_dest_->AddJointActuator(src->name(), *joint_dest,
                                       src->effort_limit());
    joint_actuators_.insert({src, joint_actuator_dest});
  }

  const std::map<ModelInstanceIndex, ModelInstanceIndex>& model_instances()
      const {
    return model_instances_;
  }

 private:
  const MultibodyPlant<double>* plant_src_;
  MultibodyPlant<double>* plant_dest_;
  MultibodyPlantElements builtins_src_;
  std::map<const Body<double>*, const Body<double>*> bodies_;
  std::map<const Frame<double>*, const Frame<double>*> frames_;
  std::map<const Joint<double>*, const Joint<double>*> joints_;
  std::map<const JointActuator<double>*, const JointActuator<double>*>
      joint_actuators_;
  std::map<ModelInstanceIndex, ModelInstanceIndex> model_instances_;
};

// TODO(azeey) implement this.
void CheckSubgraphInvariants(const MultibodyPlantElements&) {}

ModelInstanceIndex GetOrCreateModelInstanceByName(
    MultibodyPlant<double>* plant, const std::string& model_name) {
  if (plant->HasModelInstanceNamed(model_name)) {
    return plant->GetModelInstanceByName(model_name);
  }
  return plant->AddModelInstance(model_name);
}

ModelInstanceIndex ModelInstanceRemapSameName(
    const MultibodyPlant<double>& plant_src,
    ModelInstanceIndex model_instance_src, MultibodyPlant<double>* plant_dest) {
  const std::string name = plant_src.GetModelInstanceName(model_instance_src);
  return GetOrCreateModelInstanceByName(plant_dest, name);
}

std::string FrameNameRenamSameName(const MultibodyPlant<double>&,
                                   const Frame<double>& frame) {
  return frame.name();
}

class MultibodyPlantSubgraph {
 public:
  MultibodyPlantSubgraph() = default;
  explicit MultibodyPlantSubgraph(MultibodyPlantElements elem)
      : elem_src_(std::move(elem)) {
    CheckSubgraphInvariants(elem_src_);
  }

  using RemapFuncT = std::function<ModelInstanceIndex(
      const MultibodyPlant<double>&, ModelInstanceIndex,
      MultibodyPlant<double>*)>;

  MultibodyPlantElementsMap AddTo(
      MultibodyPlant<double>* plant_dest,
      RemapFuncT model_instance_remap = ModelInstanceRemapSameName,
      FrameNameRemapFuncT frame_name_remap = FrameNameRenamSameName) const {
    const auto *plant_src = &elem_src_.plant();
    MultibodyPlantElementsMap src_to_dest(plant_src, plant_dest);

    // Remap and register model instances.
    for (const auto& model_instance_src : elem_src_.model_instances()) {
      auto model_instance_dest =
          model_instance_remap(*plant_src, model_instance_src, plant_dest);
      src_to_dest.RegisterModelInstance(model_instance_src,
                                        model_instance_dest);
    }

    // Register world body and frame if we're using that source model
    // instance.
    if (src_to_dest.model_instances().find(world_model_instance()) !=
        src_to_dest.model_instances().end()) {
      src_to_dest.RegisterWorldBodyAndFrame();
    }

    // Copy bodies
    for (const auto& body_src : elem_src_.bodies()) {
      src_to_dest.CopyBody(body_src);
    }

    // Copy frames
    for (const auto& frame_src : elem_src_.frames()) {
      src_to_dest.CopyFrame(frame_src, frame_name_remap);
    }

    for (const auto& joint_src : elem_src_.joints()) {
      src_to_dest.CopyJoint(joint_src);
    }

    for (const auto& joint_actuator_src : elem_src_.joint_actuators()) {
      src_to_dest.CopyJointActuator(joint_actuator_src);
    }

    // TODO(azeey) Copy geometries (if applicable).
    // TODO(azeey) Apply policies
    return src_to_dest;
  }

  const MultibodyPlantElements& elements() const { return elem_src_; }

 private:
  MultibodyPlantElements elem_src_;
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake

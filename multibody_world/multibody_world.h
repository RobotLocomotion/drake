/** @file
 Provides functions to create a Diagram from a MultibodyPlant and a SceneGraph,
 allowing this diagram to be simulated (in time) and geometric queries to be
 posed, e.g., for planning applications.
*/

#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {

/// @cond
// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBW_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)
/// @endcond

/** Extends a Diagram in order to wrap and connect a MultibodyPlant and a
 SceneGraph, thereby allowing users to easily pose geometric queries for
 arbitrary multibody configurations. All ports from SceneGraph and
 MultibodyPlant are exported for convenience. Functionality for connecting
 to DrakeVisualizer is also provided.

 Like MultibodyPlant, MultibodyWorld must be constructed in two
 phases: construction and finalization. This two-phase construction is necessary
 so that the MultibodyPlant can be initialized in the requisite manner.
 Connection to DrakeVisualizer requires yet another step. A sketch of code for
 the typical process follows:
 @code
 systems::DiagramBuilder<double> builder;

 auto& mbw = *builder.AddSystem<MultibodyWorld<double>>();
 auto& mb_plant = mbw.mutable_multibody_plant();

 // Make and add the cart_pole model.
 AddModelFromSdfFile(filename, &mb_plant, &mbw.mutable_scene_graph());

 // Add gravity to the model.
 mb_plant.AddForceElement<UniformGravityFieldElement>(
     -9.81 * Vector3<double>::UnitZ());

 // Now the model is complete.
 mbw.Finalize();

 // We can now connect to DrakeVisualizer.
 mbw.ConnectDrakeVisualizer(&builder);
 @endcode
 */
template <class T>
class MultibodyWorld : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyWorld)

  /// Constructs the MultibodyWorld. The dynamics of the underlying
  /// MultibodyPlant will be modeled using the specified time
  /// discretization (`time_step`).
  /// @see MultibodyPlant::MultibodyPlant(double). 
  explicit MultibodyWorld(
      double time_step = 0.0);

  /// Gets a constant reference to the MultibodyPlant. This function can be
  /// called pre-Finalize.
  const multibody::multibody_plant::MultibodyPlant<T>& multibody_plant() const {
    return *multibody_plant_;
  }

  /// Gets a mutable reference to the MultibodyPlant. This function can be
  /// called pre-Finalize.
  multibody::multibody_plant::MultibodyPlant<T>& mutable_multibody_plant() {
    return *multibody_plant_;
  }

  /// Gets a constant reference to the SceneGraph. This function can be
  /// called pre-Finalize.
  const geometry::SceneGraph<T>& scene_graph() const { return *scene_graph_; }

  /// Gets a mutable reference to the SceneGraph. This function can be
  /// called pre-Finalize.
  geometry::SceneGraph<T>& mutable_scene_graph() { return *scene_graph_; }

  /// Gets the exported port of the same name from SceneGraph.
  /// @see SceneGraph::get_pose_bundle_output_port() for further documentation
  ///      (including preconditions).
  /// @pre Finalize() was already called on `this` system.
  const systems::OutputPort<T>& get_pose_bundle_output_port() const {
    DRAKE_MBW_THROW_IF_NOT_FINALIZED();
    return this->get_output_port(pose_bundle_output_port_);
  }

  /// Gets the exported port of the same name from MultibodyPlant.
  /// @see MultibodyPlant::get_actuation_input_port() for further documentation
  ///      (including preconditions).
  /// @pre Finalize() was already called on `this` system.
  const systems::InputPort<T>& get_actuation_input_port() const {
    DRAKE_MBW_THROW_IF_NOT_FINALIZED();
    if (instance_actuation_ports_.size() != 1) {
      throw std::logic_error(
          "Requested actuation input port when there are "
          + std::to_string(instance_actuation_ports_.size())
          + " (rather than exactly one) input ports");
    }
    return this->get_input_port(
        instance_actuation_ports_.front());
  }

  /// Gets the exported port of the same name from MultibodyPlant.
  /// @see MultibodyPlant::get_actuation_input_port(ModelInstanceIndex) for
  ///      further documentation (including preconditions).
  /// @pre Finalize() was already called on `this` system.
  const systems::InputPort<T>& get_actuation_input_port(
      multibody::ModelInstanceIndex model_instance) const {
    CheckModelInstanceIsValid(model_instance);
    return this->get_input_port(
        instance_continuous_state_output_ports_.at(model_instance));
  }

  /// Gets the exported port of the same name from MultibodyPlant.
  /// @see MultibodyPlant::get_continuous_state_output_port() for further
  ///      documentation (including preconditions).
  /// @pre Finalize() was already called on `this` system.
  const systems::OutputPort<T>& get_continuous_state_output_port() const {
    DRAKE_MBW_THROW_IF_NOT_FINALIZED();
    return this->get_output_port(continuous_state_output_port_);
  }

  /// Gets the exported port of the same name from MultibodyPlant.
  /// @see MultibodyPlant::get_continuous_state_output_port(ModelInstanceIndex)
  ///      for further documentation (including preconditions).
  /// @pre Finalize() was already called on `this` system.
  const systems::OutputPort<T>& get_continuous_state_output_port(
      multibody::ModelInstanceIndex model_instance) const {
    DRAKE_MBW_THROW_IF_NOT_FINALIZED();
    CheckModelInstanceIsValid(model_instance);
    return this->get_output_port(
        instance_continuous_state_output_ports_.at(model_instance));
  }

  /// Gets the exported port of the same name from MultibodyPlant.
  /// @see MultibodyPlant::get_generalized_contact_forces_output_port() for
  ///      further documentation (including preconditions).
  /// @pre Finalize() was already called on `this` system.
  const systems::OutputPort<T>& get_generalized_contact_forces_output_port(
      multibody::ModelInstanceIndex model_instance) const {
    DRAKE_MBW_THROW_IF_NOT_FINALIZED();
    CheckModelInstanceIsValid(model_instance);
    return this->get_output_port(
        instance_generalized_contact_forces_output_ports_.at(model_instance));
  }

  /// Gets the exported port of the same name from MultibodyPlant.
  /// @see MultibodyPlant::get_contact_results_output_port() for further
  ///      documentation (including preconditions).
  /// @pre Finalize() was already called on `this` system.
  const systems::OutputPort<T>& get_contact_results_output_port() const {
    DRAKE_MBW_THROW_IF_NOT_FINALIZED();
    return this->get_output_port(contact_results_port_);
  }

  /// Users *must* call Finalize() after making any additions to the
  /// MultibodyPlant and before using this class in the Systems framework.
  /// This should be called exactly once.
  ///
  /// @see multibody::multibody_plant::MultibodyPlant<T>::Finalize()
  void Finalize();

  /// Determines whether this system has been finalized (via a call to
  /// Finalize()).
  bool is_finalized() const {
    return finalized_;
  }

  /// Gets the MultibodyPlant context from a Diagram containing `this`.
  /// @pre Finalize() was already called on `this` system.
  systems::Context<T>& GetMutableMultibodyPlantContext(
      systems::Diagram<T>* diagram, systems::Context<T>* diagram_context) {
    DRAKE_MBW_THROW_IF_NOT_FINALIZED();

    auto& mbw_context = diagram->GetMutableSubsystemContext(
        *this, diagram_context);
    return this->GetMutableSubsystemContext(*multibody_plant_, &mbw_context);
  }

  /// Connects this Diagram to the DrakeVisualizer. This method must be called
  /// _during_ Diagram building and uses the given `builder` to add relevant
  /// subsystems and connections. Code is modified from
  /// geometry::ConnectDrakeVisualizer(), and provides identical functionality.
  /// @see geometry::ConnectDrakeVisualizer() for a description of the overall
  ///      process.
  /// @param builder  The diagram builder being used to construct the  Diagram.
  /// @param lcm  An optional lcm interface through which lcm messages
  ///        will be dispatched. Will be allocated internally if
  ///        none is supplied.
  template <typename U = T>
  std::enable_if_t<std::is_same<U, double>::value,
                   systems::lcm::LcmPublisherSystem*> ConnectDrakeVisualizer(
      systems::DiagramBuilder<T>* builder,
      lcm::DrakeLcmInterface* lcm_optional = nullptr) const {
    using systems::lcm::LcmPublisherSystem;
    using systems::lcm::Serializer;
    using systems::rendering::PoseBundleToDrawMessage;
    DRAKE_MBW_THROW_IF_NOT_FINALIZED();

    PoseBundleToDrawMessage* converter =
        builder->template AddSystem<PoseBundleToDrawMessage>();

    LcmPublisherSystem* publisher =
        builder->template AddSystem<LcmPublisherSystem>(
            "DRAKE_VIEWER_DRAW",
            std::make_unique<Serializer<drake::lcmt_viewer_draw>>(),
            lcm_optional);
    publisher->set_publish_period(1 / 60.0);

    // The functor we create in publisher here holds a reference to scene_graph,
    // which must therefore live as long as publisher does. We can count on that
    // because scene_graph is required to be contained in builder (see method
    // documentation), along with the converter and publisher we just added.
    // Builder will transfer ownership of all of these objects to the Diagram it
    // eventually builds.
    publisher->AddInitializationMessage([this](
        const systems::Context<double>&, lcm::DrakeLcmInterface* lcm) {
      DispatchLoadMessage(*scene_graph_, lcm);
    });

    // Note that this will fail if scene_graph is not actually in builder.
    builder->Connect(this->get_pose_bundle_output_port(),
                     converter->get_input_port(0));
    builder->Connect(*converter, *publisher);

    return publisher;
  }

 private:
  void CheckModelInstanceIsValid(
      multibody::ModelInstanceIndex model_instance) const {
    DRAKE_THROW_UNLESS(model_instance.is_valid());
    DRAKE_THROW_UNLESS(model_instance <
        multibody_plant_->num_model_instances());
    DRAKE_THROW_UNLESS(multibody_plant_->tree().num_states(model_instance) > 0);
  }

  // Helper method for throwing an exception within public methods that should
  // not be called pre-finalize. The invoking method should pass its name so
  // that the error message can include that detail.
  void ThrowIfNotFinalized(const char* source_method) const {
    if (!is_finalized()) {
      throw std::logic_error(
          "Pre-finalize calls to '" + std::string(source_method) + "()' are "
              "not allowed; you must call Finalize() first.");
    }
  }

  // The builder that builds this Diagram.
  std::unique_ptr<systems::DiagramBuilder<T>> builder_;

  // The pointer to the MultibodyPlant created by `builder_`
  multibody::multibody_plant::MultibodyPlant<T>* multibody_plant_{nullptr};

  // The pointer to the SceneGraph created by `builder_`.
  geometry::SceneGraph<T>* scene_graph_{nullptr};

  // Index for the output port of ContactResults.
  systems::OutputPortIndex contact_results_port_;

  // A vector containing the index for the generalized contact forces port for
  // each model instance. This vector is indexed by ModelInstanceIndex. An
  // invalid value indicates that the model instance has no generalized
  // velocities and thus no generalized forces.
  std::vector<systems::OutputPortIndex>
      instance_generalized_contact_forces_output_ports_;

  // A vector containing actuation ports for each model instance indexed by
  // ModelInstanceIndex.  An invalid value indicates that the model instance has
  // no actuated DOFs.
  std::vector<systems::InputPortIndex> instance_actuation_ports_;

  // If only one model instance has actuated DOFs, remember it here.  If
  // multiple instances have actuated DOFs, this index will not be valid.
  systems::InputPortIndex single_actuated_port_;

  // Port for output of all continuous state from MultibodyPlant.
  systems::OutputPortIndex continuous_state_output_port_;

  // Port for PoseBundle outputs from SceneGraph.
  systems::OutputPortIndex pose_bundle_output_port_;

  // Output port for queries from SceneGraph.
  systems::OutputPortIndex query_output_port_;

  // A vector containing state output ports for each model instance indexed by
  // ModelInstanceIndex. An invalid value indicates that the model instance has
  // no state.
  std::vector<systems::OutputPortIndex> instance_continuous_state_output_ports_;

  // Whether this system has been finalized.
  bool finalized_{false};
};

template <class T>
MultibodyWorld<T>::MultibodyWorld(double time_step) {
  builder_ = std::make_unique<systems::DiagramBuilder<T>>();

  scene_graph_ = builder_->template AddSystem<geometry::SceneGraph<T>>();
  scene_graph_->set_name("scene_graph");

  multibody_plant_ =
      builder_->template AddSystem<
          multibody::multibody_plant::MultibodyPlant<T>>(time_step);
}

template <class T>
void MultibodyWorld<T>::Finalize() {
  // Verify that the system is not already finalized.
  if (is_finalized())
    throw std::logic_error("MultibodyWorld::Finalize() has already"
                               " been called");

  // Verify that a source has been registered.
  if (!multibody_plant_->get_source_id()) {
    throw std::logic_error("MultibodyPlant source ID not defined; have you "
                               "forgotten to add geometries?");
  }

  // MultibodyPlant must be finalized first.
  multibody_plant_->Finalize(scene_graph_);

  // Exports the pose bundle output and query output ports from Scene Graph.
  pose_bundle_output_port_ = builder_->ExportOutput(
      scene_graph_->get_pose_bundle_output_port());
  query_output_port_ = builder_->ExportOutput(
      scene_graph_->get_query_output_port());

  // Export the contact results port for MultibodyPlant.
  if (multibody_plant_->is_discrete()) {
    contact_results_port_ = builder_->ExportOutput(
        multibody_plant_->get_contact_results_output_port());
  }

  // Exports the model-instance based ports for MultibodyPlant.
  continuous_state_output_port_ = builder_->ExportOutput(
      multibody_plant_->get_continuous_state_output_port());
  int num_actuated_instances = 0;
  for (multibody::ModelInstanceIndex i(0);
       i < multibody_plant_->num_model_instances(); ++i) {
    if (multibody_plant_->num_actuated_dofs(i) > 0) {
      instance_actuation_ports_.push_back(builder_->ExportInput(
          multibody_plant_->get_actuation_input_port(i)));
      ++num_actuated_instances;
    }

    if (multibody_plant_->tree().num_states(i) > 0) {
      instance_continuous_state_output_ports_.push_back(builder_->
          ExportOutput(multibody_plant_->get_continuous_state_output_port(i)));
    }

    if (multibody_plant_->is_discrete()) {
      instance_generalized_contact_forces_output_ports_.push_back(
          builder_->ExportOutput(
              multibody_plant_->get_generalized_contact_forces_output_port(i)));
    }
  }

  // Create the necessary connections.
  builder_->Connect(multibody_plant_->get_geometry_poses_output_port(),
                    scene_graph_->get_source_pose_port(
                    multibody_plant_->get_source_id().value()));

  builder_->Connect(scene_graph_->get_query_output_port(),
                    multibody_plant_->get_geometry_query_input_port());

  // Construct this Diagram.
  builder_->BuildInto(this);

  // Indicate that finalization is complete.
  finalized_ = true;
}

}  // namespace drake

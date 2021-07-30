#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"

namespace drake {
namespace multibody {
namespace test {

using Eigen::Vector3d;
using math::RigidTransformd;

// This class helps reduce the amount of boiler plate needed to setup a
// multibody simulation for unit testing. It provides a minimum API to aid the
// quick prototyping of unit tests from an SDF/URDF description and to retrieve
// results relevant to contact dynamics.
class MultibodySimDriver {
 public:
  MultibodySimDriver() = default;

  // Builds a MultibodyPlant/SceneGraph description of the model described by
  // `model_file`, either in URDF or SDF format.
  // We set gravity to (0.0, 0.0, -10.0) m/s² so that within unit tests numbers
  // are simpler.
  // The underlying plant is not finalized yet so that we can modify the model
  // with access through `mutable_plant()`.
  // MultibodyPlant::Finalize() will happen at `Initialize()`.
  void BuildModel(double dt, const std::string& model_file);

  // Prepares the driver for simulation. In particualar, we call
  // MultibodyPlant::Finalize() and Simulator::Initialize().
  void Initialize();

  // @pre BuildModel() must have been called.
  const MultibodyPlant<double>& plant() const {
    DRAKE_DEMAND(plant_ != nullptr);
    return *plant_;
  }

  // @pre BuildModel() must have been called.
  MultibodyPlant<double>& mutable_plant() {
    DRAKE_DEMAND(plant_ != nullptr);
    return *plant_;
  }

  // @pre Initialize() must have been called.
  const systems::Context<double>& plant_context() const {
    DRAKE_DEMAND(initialized_);
    return *plant_context_;
  }

  // @pre Initialize() must have been called.
  systems::Context<double>& mutable_plant_context() {
    DRAKE_DEMAND(initialized_);
    return *plant_context_;
  }

  // Helper to add a model of the ground with point contact parameters
  // `stiffness` and `dissipation`. For discrete contact solvers (the realm of
  // this driver), only `dynamic_friction` must be provided.
  void AddGround(double stiffness, double damping, double dynamic_friction);

  // Helper to retrieve the coefficients of dynamic friction for a given `body`.
  // @returns a vector with as many geometries associated to this body, each
  // containing the friction coefficient for that geometry.
  // TODO(amcastro-tri): consider returning a std::pair<double, GeometryId>
  // when/if needed.
  std::vector<double> GetDynamicFrictionCoefficients(
      const Body<double>& body) const;

  // Retrieves contact results by evaluating MultibodyPlant's port.
  // Unless results are already cached, this computation will trigger the
  // expensive computation required to find out contact impulses and the
  // corresponding velocity updates.
  // @pre We called Initialize().
  const ContactResults<double>& GetContactResults() const {
    DRAKE_DEMAND(initialized_);
    const ContactResults<double>& contact_results =
        plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
            *plant_context_);
    return contact_results;
  }

  // This method fixes MultibodyPlant::get_applied_spatial_force_input_port() so
  // that a constant force `f_Bo_W` is applied on `body`, at its origin Bo. The
  // force is expressed in the world frame.
  // @pre We called Initialize().
  void FixAppliedForce(const Body<double>& body, const Vector3d& f_Bo_W) {
    DRAKE_DEMAND(initialized_);
    std::vector<ExternallyAppliedSpatialForce<double>> forces(1);
    forces[0].body_index = body.index();
    forces[0].p_BoBq_B = Vector3d::Zero();
    forces[0].F_Bq_W = SpatialForce<double>(Vector3d(0.0, 0.0, 0.0), f_Bo_W);
    plant_->get_applied_spatial_force_input_port().FixValue(plant_context_,
                                                            forces);
  }

 private:
  void SetPointContactParameters(const Body<double>& body, double stiffness,
                                 double damping);

  // Helper to get an inspector pre-initialization (no context available) or
  // post-initialization (a context is available).
  const geometry::SceneGraphInspector<double>& GetInspector() const;

  MultibodyPlant<double>* plant_{nullptr};
  geometry::SceneGraph<double>* scene_graph_{nullptr};
  std::unique_ptr<systems::Diagram<double>> diagram_;
  systems::Context<double>* diagram_context_{nullptr};
  systems::Context<double>* plant_context_{nullptr};
  systems::Context<double>* scene_graph_context_{nullptr};
  std::unique_ptr<systems::DiscreteValues<double>> discrete_values_;
  std::unique_ptr<systems::Simulator<double>> simulator_;
  bool initialized_{false};
  systems::DiagramBuilder<double> builder_;  // only used for building.
};

}  // namespace test
}  // namespace multibody
}  // namespace drake

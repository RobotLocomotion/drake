#pragma once

#include <memory>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/multibody/plant/contact_results.h"

namespace drake {
namespace multibody {
namespace test {

using Eigen::Vector3d;
using math::RigidTransformd;

class ContactSolverDriver {
 public:
  ContactSolverDriver() = default;

  void BuildModel(double dt, const std::string& model_file);

  void Initialize();

  const MultibodyPlant<double>& plant() const {
    DRAKE_DEMAND(plant_);
    return *plant_;
  }

  MultibodyPlant<double>& mutable_plant() {
    DRAKE_DEMAND(plant_);
    return *plant_;
  }

  const systems::Context<double>& plant_context() const {
    DRAKE_DEMAND(initialized_);
    return *plant_context_;
  }

  systems::Context<double>& mutable_plant_context() {
    DRAKE_DEMAND(plant_context_ != nullptr);
    return *plant_context_;
  }

  void AddGround(double stiffness, double damping, double dynamic_friction);

  void SetPointContactParameters(const Body<double>& body, double stiffness,
                                 double damping);  

  std::vector<std::pair<double, double>> GetPointContactComplianceParameters(
      const Body<double>& body);

  void SetDynamicFrictionCoefficient(const Body<double>& body, double mu_d);

  std::vector<double> GetDynamicFrictionCoefficients(const Body<double>& body) const;

  // For viz.
  void Publish() const { diagram_->Publish(*diagram_context_); }

  void AdvanceNumSteps(int num_steps) {
    const double time = diagram_context_->get_time();
    const double dt = plant_->time_step();
    const double next_time = time + dt * num_steps;
    simulator_->AdvanceTo(next_time);

    //    diagram_->CalcDiscreteVariableUpdates(*diagram_context_,
    //                                        discrete_values_.get());
    // const VectorX<double> xnext =
    // discrete_values_->get_vector().CopyToVector();
  }

  const ContactResults<double>& GetContactResults() const {
    DRAKE_DEMAND(plant_);
    DRAKE_DEMAND(diagram_context_);
    const ContactResults<double>& contact_results =
        plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
            *plant_context_);
    return contact_results;
  }

  void FixAppliedForce(const Body<double>& body, const Vector3d& f_Bo_W) {
    DRAKE_DEMAND(plant_);
    DRAKE_DEMAND(plant_context_);
    std::vector<ExternallyAppliedSpatialForce<double>> forces(1);
    forces[0].body_index = body.index();
    forces[0].p_BoBq_B = Vector3d::Zero();
    forces[0].F_Bq_W = SpatialForce<double>(Vector3d(0.0, 0.0, 0.0), f_Bo_W);
    plant_->get_applied_spatial_force_input_port().FixValue(plant_context_,
                                                            forces);
  }

 private:
  // Creates the system's context and returns a mutable reference to the
  // plant's context.
  //systems::Context<double>& CreateDefaultContext() {
   // diagram_context_ = diagram_->CreateDefaultContext();
    //plant_context_ =
      //  &plant_->GetMyMutableContextFromRoot(diagram_context_.get());
    //return *plant_context_;
  //}

  // Helpers to get an inspector pre-initialization (no context available) or
  // post-initialization (a context is available).
  const geometry::SceneGraphInspector<double>& GetInspector() const;
  const geometry::SceneGraphInspector<double>& GetInspectorToMutableState();

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

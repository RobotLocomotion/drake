#pragma once

#include <memory>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace test {

using Eigen::Vector3d;
using math::RigidTransformd;

class ContactSolverDriver {
 public:
  ContactSolverDriver() = default;

  void BuildModel(double dt, const std::string& model_file);

  

  const MultibodyPlant<double>& plant() const {
    DRAKE_DEMAND(plant_);
    return *plant_;
  }

  MultibodyPlant<double>& mutable_plant() {
    DRAKE_DEMAND(plant_);
    return *plant_;
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

 private:
  // Creates the system's context and returns a mutable reference to the
  // plant's context.
  //systems::Context<double>& CreateDefaultContext() {
   // diagram_context_ = diagram_->CreateDefaultContext();
    //plant_context_ =
      //  &plant_->GetMyMutableContextFromRoot(diagram_context_.get());
    //return *plant_context_;
  //}

  MultibodyPlant<double>* plant_{nullptr};
  geometry::SceneGraph<double>* scene_graph_{nullptr};
  std::unique_ptr<systems::Diagram<double>> diagram_;
  systems::Context<double>* diagram_context_{nullptr};  
  systems::Context<double>* plant_context_{nullptr};
  systems::Context<double>* scene_graph_context_{nullptr};
  std::unique_ptr<systems::DiscreteValues<double>> discrete_values_;
  std::unique_ptr<systems::Simulator<double>> simulator_;
};

}  // namespace test
}  // namespace multibody
}  // namespace drake

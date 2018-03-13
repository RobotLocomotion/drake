#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_system.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/benchmarks/pendulum/make_pendulum_plant.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {

using geometry::GeometrySystem;
using geometry::SourceId;
using lcm::DrakeLcm;
using multibody::benchmarks::pendulum::MakePendulumPlant;
using multibody::benchmarks::pendulum::PendulumParameters;
using multibody::multibody_plant::MultibodyPlant;
using multibody::RevoluteJoint;
using systems::BasicVector;
using systems::Context;
using systems::ImplicitEulerIntegrator;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::Serializer;
using systems::rendering::PoseBundleToDrawMessage;
using systems::RungeKutta3Integrator;
using systems::SemiExplicitEulerIntegrator;

namespace examples {
namespace multibody {
namespace pendulum {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_string(integration_scheme, "runge_kutta3",
              "Integration scheme to be used. Available options are:"
              "'runge_kutta3','implicit_euler','semi_explicit_euler'");

template <typename T>
class PendulumEnergyShapingController : public systems::LeafSystem<T> {
 public:
  explicit PendulumEnergyShapingController(const PendulumParameters& params) :
      params_(params) {
    model_ = MakePendulumPlant(params);
    // Allocate a model Context to work with.
    context_ = model_->CreateDefaultContext();
    pin_ = &model_->template GetJointByName<RevoluteJoint>(
        params.pin_joint_name());
    // Sanity check the sizes.
    DRAKE_DEMAND(model_->num_multibody_states() == 2);
    DRAKE_DEMAND(model_->num_actuated_dofs() == 1);
    this->DeclareVectorInputPort(BasicVector<T>(
        model_->num_multibody_states()));
    this->DeclareVectorOutputPort(BasicVector<T>(model_->num_actuated_dofs()),
                                  &PendulumEnergyShapingController::CalcTau);
  }

 private:
  void CalcTau(const systems::Context<T>& context,
               BasicVector<T>* output) const {
    const auto& xc = this->EvalEigenVectorInput(context, 0);
    context_->get_mutable_continuous_state().SetFromVector(xc);

    // context_->get_state().SetFrom(state); // maybe shorter/cleaner??
    //context_->get_mutable_continuous_state().
      //  get_mutable_vector().SetFrom(*state);

    // Pendulum energy shaping from Section 3.5.2 of
    // http://underactuated.csail.mit.edu/underactuated.html?chapter=3
    using std::pow;
    // Desired energy is slightly more than the energy at the top (want to pass
    // through the upright with non-zero velocity).
    const T desired_energy = 1.1 * params_.m() * params_.g() * params_.l();
//    const T current_energy =
  //      0.5 * params_.m() * pow(params_.l() * thetadot, 2) -
    //        params_.m() * params_.g() * params_.l() * cos(theta);
    const T current_energy = model_->CalcTotalEnergy(*context_);
    const double kEnergyFeedbackGain = .1;
    // TODO(amcastro-tri): Add damping to the model.
    const T& thetadot = pin_->get_angular_rate(*context_);
    const T tau =
        kEnergyFeedbackGain * thetadot * (desired_energy - current_energy);
    output->SetAtIndex(0, tau);
  }
  // The controller's model of the plant.
  PendulumParameters params_;
  std::unique_ptr<MultibodyPlant<T>> model_;
  const RevoluteJoint<T>* pin_{nullptr};
  std::unique_ptr<Context<T>> context_;
};

int do_main() {
  systems::DiagramBuilder<double> builder;

  GeometrySystem<double>& geometry_system =
      *builder.AddSystem<GeometrySystem>();
  geometry_system.set_name("geometry_system");

  // The model's parameters:
  PendulumParameters parameters;

  // Define simulation parameters:
  // Compute a reference time scale to set reasonable values for time step and
  // simulation time.
  const double reference_time_scale =
      2.0 * M_PI * sqrt(parameters.l() / parameters.g());

  // Define a reasonable maximum time step based off the expected dynamics's
  // time scales.
  const double max_time_step = reference_time_scale / 100;

  // Simulate about five periods of oscillation.
  const double simulation_time = 5.0 * reference_time_scale;

  // The target accuracy determines the size of the actual time steps taken
  // whenever a variable time step integrator is used.
  const double target_accuracy = 0.001;

  MultibodyPlant<double>& pendulum =
      *builder.AddSystem(MakePendulumPlant(parameters, &geometry_system));
  const RevoluteJoint<double>& pin =
      pendulum.GetJointByName<RevoluteJoint>(parameters.pin_joint_name());

  auto controller =
      builder.AddSystem<PendulumEnergyShapingController>(parameters);
  controller->set_name("controller");
  builder.Connect(pendulum.get_state_output_port(),
                  controller->get_input_port(0));
  builder.Connect(controller->get_output_port(0),
                  pendulum.get_actuation_input_port());

  // Boilerplate used to connect the plant to a GeometrySystem for
  // visualization.
  DrakeLcm lcm;
  const PoseBundleToDrawMessage& converter =
      *builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem& publisher =
      *builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher.set_publish_period(1 / 60.0);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!pendulum.get_source_id());

  builder.Connect(
      pendulum.get_geometry_ids_output_port(),
      geometry_system.get_source_frame_id_port(
          pendulum.get_source_id().value()));
  builder.Connect(
      pendulum.get_geometry_poses_output_port(),
      geometry_system.get_source_pose_port(pendulum.get_source_id().value()));

  builder.Connect(geometry_system.get_pose_bundle_output_port(),
                  converter.get_input_port(0));
  builder.Connect(converter, publisher);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(geometry_system);

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  auto diagram_context = diagram->CreateDefaultContext();
  systems::Context<double>& pendulum_context =
      diagram->GetMutableSubsystemContext(pendulum, diagram_context.get());
  pin.set_angle(&pendulum_context,  M_PI / 3.0);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  systems::IntegratorBase<double>* integrator{nullptr};
  if (FLAGS_integration_scheme == "implicit_euler") {
    integrator =
        simulator.reset_integrator<ImplicitEulerIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    integrator =
        simulator.reset_integrator<RungeKutta3Integrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    integrator =
        simulator.reset_integrator<SemiExplicitEulerIntegrator<double>>(
            *diagram, max_time_step, &simulator.get_mutable_context());
  } else {
    throw std::runtime_error(
        "Integration scheme '" + FLAGS_integration_scheme +
        "' not supported for this example.");
  }
  integrator->set_maximum_step_size(max_time_step);

  // Error control is only supported for variable time step integrators.
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(target_accuracy);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(simulation_time);

  // Some sanity checks:
  if (FLAGS_integration_scheme == "semi_explicit_euler") {
    DRAKE_DEMAND(integrator->get_fixed_step_mode() == true);
  }

  // Checks for variable time step integrators.
  if (!integrator->get_fixed_step_mode()) {
    DRAKE_DEMAND(integrator->get_largest_step_size_taken() <= max_time_step);
    DRAKE_DEMAND(integrator->get_smallest_adapted_step_size_taken() <=
        integrator->get_largest_step_size_taken());
    DRAKE_DEMAND(
        integrator->get_num_steps_taken() >= simulation_time / max_time_step);
  }

  // Checks for fixed time step integrators.
  if (integrator->get_fixed_step_mode()) {
    DRAKE_DEMAND(integrator->get_num_derivative_evaluations() ==
        integrator->get_num_steps_taken());
    DRAKE_DEMAND(
        integrator->get_num_step_shrinkages_from_error_control() == 0);
  }

  // We made a good guess for max_time_step and therefore we expect no
  // failures when taking a time step.
  DRAKE_DEMAND(integrator->get_num_substep_failures() == 0);
  DRAKE_DEMAND(
      integrator->get_num_step_shrinkages_from_substep_failures() == 0);

  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple pendulum demo using Drake's MultibodyPlant. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::pendulum::do_main();
}

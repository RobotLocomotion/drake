#include "drake/examples/kuka_iiwa_arm/kuka_torque_controller.h"

#include <memory>

#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/adder.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using drake::systems::kVectorValued;
using drake::systems::Adder;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::LeafSystem;
using drake::systems::controllers::PidController;

template <typename T>
class StateDependentDamper : public LeafSystem<T> {
 public:
  StateDependentDamper(const multibody::MultibodyPlant<T>& plant,
                       const VectorX<double>& stiffness,
                       const VectorX<double>& damping_ratio)
      : plant_(plant), stiffness_(stiffness), damping_ratio_(damping_ratio) {
    const int num_q = plant_.num_positions();
    const int num_v = plant_.num_velocities();
    const int num_x = num_q + num_v;

    DRAKE_DEMAND(stiffness.size() == num_v);
    DRAKE_DEMAND(damping_ratio.size() == num_v);

    this->DeclareInputPort(systems::kUseDefaultName, kVectorValued, num_x);
    this->DeclareVectorOutputPort(systems::kUseDefaultName, num_v,
                                  &StateDependentDamper<T>::CalcTorque);
    // Make context with default parameters.
    plant_context_ = plant_.CreateDefaultContext();
  }

 private:
  const multibody::MultibodyPlant<T>& plant_;
  const VectorX<double> stiffness_;
  const VectorX<double> damping_ratio_;

  // This context is used solely for setting generalized positions and
  // velocities in multibody_plant_.
  std::unique_ptr<Context<T>> plant_context_;

  /**
   * Computes joint level damping forces by computing the damping ratio for each
   * joint independently as if all other joints were fixed. Note that the
   * effective inertia of a joint, when all of the other joints are fixed, is
   * given by the corresponding diagonal entry of the mass matrix. The critical
   * damping gain for the i-th joint is given by 2*sqrt(M(i,i)*stiffness(i)).
   */
  void CalcTorque(const Context<T>& context, BasicVector<T>* torque) const {
    const Eigen::VectorXd& x = this->EvalVectorInput(context, 0)->value();
    plant_.SetPositionsAndVelocities(plant_context_.get(), x);

    const int num_v = plant_.num_velocities();
    Eigen::MatrixXd H(num_v, num_v);
    plant_.CalcMassMatrixViaInverseDynamics(*plant_context_, &H);

    // Compute critical damping gains and scale by damping ratio. Use Eigen
    // arrays (rather than matrices) for elementwise multiplication.
    Eigen::ArrayXd temp =
        H.diagonal().array() * stiffness_.array();
    Eigen::ArrayXd damping_gains = 2 * temp.sqrt();
    damping_gains *= damping_ratio_.array();

    // Compute damping torque.
    Eigen::VectorXd v = x.tail(plant_.num_velocities());
    torque->get_mutable_value() = -(damping_gains * v.array()).matrix();
  }
};

}  // namespace

template <typename T>
KukaTorqueController<T>::KukaTorqueController(
    const multibody::MultibodyPlant<T>& plant,
    const VectorX<double>& stiffness,
    const VectorX<double>& damping)
    : plant_(plant) {

  DiagramBuilder<T> builder;
  DRAKE_DEMAND(plant_.num_positions() == stiffness.size());
  DRAKE_DEMAND(plant_.num_positions() == damping.size());

  const int dim = plant_.num_positions();

  /*
  torque_in ----------------------------
                                       |
  (q, v)    ------>|Gravity Comp|------+---> torque_out
               |                      /|
               --->|Damping|---------- |
               |                       |
               --->| Virtual Spring |---
  (q*, v*)  ------>|PID with kd=ki=0|
  */

  // Adds gravity compensator.
  using drake::systems::controllers::InverseDynamics;
  auto gravity_comp =
      builder.template AddSystem<InverseDynamics<T>>(
          &plant_, InverseDynamics<T>::kGravityCompensation);

  // Adds virtual springs.
  Eigen::VectorXd kd(dim);
  Eigen::VectorXd ki(dim);
  kd.setZero();
  ki.setZero();
  auto spring = builder.template AddSystem<PidController<T>>(stiffness, kd, ki);

  // Adds virtual damper.
  auto damper = builder.template AddSystem<StateDependentDamper<T>>(
      plant_, stiffness, damping);

  // Adds an adder to sum the gravity compensation, spring, damper, and
  // feedforward torque.
  auto adder = builder.template AddSystem<Adder<T>>(4, dim);

  // Connects the gravity compensation, spring, and damper torques to the adder.
  builder.Connect(gravity_comp->get_output_port(0), adder->get_input_port(1));
  builder.Connect(spring->get_output_port(0), adder->get_input_port(2));
  builder.Connect(damper->get_output_port(0), adder->get_input_port(3));

  // Exposes the estimated state port.
  // Connects the estimated state to the gravity compensator.
  input_port_index_estimated_state_ = builder.ExportInput(
      gravity_comp->get_input_port_estimated_state());

  // Connects the estimated state to the spring.
  builder.ConnectInput(input_port_index_estimated_state_,
                       spring->get_input_port_estimated_state());

  // Connects the estimated state to the damper.
  builder.ConnectInput(input_port_index_estimated_state_,
                       damper->get_input_port(0));

  // Exposes the desired state port.
  input_port_index_desired_state_ =
      builder.ExportInput(spring->get_input_port_desired_state());

  // Exposes the commanded torque port.
  input_port_index_commanded_torque_ =
      builder.ExportInput(adder->get_input_port(0));

  // Exposes controller output.
  output_port_index_control_ = builder.ExportOutput(adder->get_output_port());

  builder.BuildInto(this);
}

template class KukaTorqueController<double>;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/examples/geometry_world/gen/bouncing_ball_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace bouncing_ball {

/** A model of a bouncing ball with Hunt-Crossley compliant contact model.

 @tparam T The vector element type, which must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:
 - double */
template <typename T>
class BouncingBallPlant : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BouncingBallPlant)

  explicit BouncingBallPlant(const Vector2<T>& init_position);
  ~BouncingBallPlant() override;

  using MyContext = systems::Context<T>;
  using MyContinuousState = systems::ContinuousState<T>;
  using MyOutput = systems::SystemOutput<T>;

  /** Returns the port to output state. */
  const systems::OutputPort<T>& get_state_output_port() const;

  void set_z(MyContext* context, const T& z) const {
    get_mutable_state(context).set_z(z);
  }

  void set_zdot(MyContext* context, const T& zdot) const {
    get_mutable_state(context).set_zdot(zdot);
  }

  /** Mass in kg. */
  T m() const { return m_; }

  /** Stiffness constant. */
  T k() const {return k_; }

  /** Hunt-Crossley's dissipation factor. */
  T d() const {return d_; }

  /** Gravity in m/s^2. */
  T g() const { return g_; }

 private:
  // Callback for writing the state vector to an output.
  void CopyStateToOutput(const MyContext& context,
                         BouncingBallVector<T>* state_output_vector) const;

  void DoCalcTimeDerivatives(const MyContext& context,
                             MyContinuousState* derivatives) const override;

  static const BouncingBallVector<T>& get_state(
      const MyContinuousState& cstate) {
    return dynamic_cast<const BouncingBallVector<T>&>(cstate.get_vector());
  }

  static BouncingBallVector<T>& get_mutable_state(
      MyContinuousState* cstate) {
    return dynamic_cast<BouncingBallVector<T>&>(cstate->get_mutable_vector());
  }

  BouncingBallVector<T>* get_mutable_state_output(MyOutput *output) const {
    return dynamic_cast<BouncingBallVector<T>*>(
        output->GetMutableVectorData(state_port_));
  }

  static const BouncingBallVector<T>& get_state(const MyContext& context) {
    return get_state(context.get_continuous_state());
  }

  static BouncingBallVector<T>& get_mutable_state(MyContext* context) {
    return get_mutable_state(&context->get_mutable_continuous_state());
  }

  const Vector2<T> init_position_;

  int state_port_;

  const double diameter_{0.1};  // Ball diameter, just for visualization.
  const double m_{0.1};   // kg
  const double g_{9.81};  // m/s^2
  // Stiffness constant [N/m]. Calculated so that under its own weight the ball
  // penetrates the plane by 1 mm when the contact force and gravitational
  // force are in equilibrium.
  const double k_{m_* g_ / 0.001};
  // Hunt-Crossley's dissipation factor.
  const double d_{0.0};  // s/m
};

}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake

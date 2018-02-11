#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/examples/geometry_world/gen/bouncing_ball_vector.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_system.h"
#include "drake/geometry/query_object.h"
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

  BouncingBallPlant(geometry::SourceId source_id,
                    geometry::GeometrySystem<T>* geometry_system,
                    const Vector2<T>& init_position);
  ~BouncingBallPlant() override;

  using MyContext = systems::Context<T>;
  using MyContinuousState = systems::ContinuousState<T>;
  using MyOutput = systems::SystemOutput<T>;

  const systems::InputPortDescriptor<T>& get_geometry_query_input_port() const;
  /** Returns the port to output state. */
  const systems::OutputPort<T>& get_state_output_port() const;
  const systems::OutputPort<T>& get_geometry_id_output_port() const;
  const systems::OutputPort<T>& get_geometry_pose_output_port() const;

  void set_z(MyContext* context, const T& z) const {
    get_mutable_state(context).set_z(z);
  }

  void set_zdot(MyContext* context, const T& zdot) const {
    get_mutable_state(context).set_zdot(zdot);
  }

  /** Mass in kg. */
  T m() const { return m_; }

  /** Stiffness constant. */
  T k() const { return k_; }

  /** Hunt-Crossley's dissipation factor. */
  T d() const { return d_; }

  /** Gravity in m/s^2. */
  T g() const { return g_; }

 protected:
  // The single input (geometry query) only impacts time derivatives and does
  // not affect any output port. So, there are no direct feedthrouhgs.
  optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return false;
  }

 private:
  // Callback for writing the state vector to an output.
  void CopyStateToOutput(const MyContext& context,
                         BouncingBallVector<T>* state_output_vector) const;

  // Allocate the frame pose set output port value.
  geometry::FramePoseVector<T> AllocateFramePoseOutput(
      const MyContext& context) const;

  // Calculate the frame pose set output port value.
  void CalcFramePoseOutput(const MyContext& context,
                           geometry::FramePoseVector<T>* poses) const;

  // Allocate the id output.
  geometry::FrameIdVector AllocateFrameIdOutput(const MyContext& context) const;
  // Calculate the id output.
  void CalcFrameIdOutput(const MyContext& context,
                         geometry::FrameIdVector* id_set) const;

  void DoCalcTimeDerivatives(const MyContext& context,
                             MyContinuousState* derivatives) const override;

  static const BouncingBallVector<T>& get_state(
      const MyContinuousState& cstate) {
    return dynamic_cast<const BouncingBallVector<T>&>(cstate.get_vector());
  }

  static BouncingBallVector<T>& get_mutable_state(MyContinuousState* cstate) {
    return dynamic_cast<BouncingBallVector<T>&>(cstate->get_mutable_vector());
  }

  BouncingBallVector<T>* get_mutable_state_output(MyOutput* output) const {
    return dynamic_cast<BouncingBallVector<T>*>(
        output->GetMutableVectorData(state_port_));
  }

  static const BouncingBallVector<T>& get_state(const MyContext& context) {
    return get_state(context.get_continuous_state());
  }

  static BouncingBallVector<T>& get_mutable_state(MyContext* context) {
    return get_mutable_state(&context->get_mutable_continuous_state());
  }

  geometry::SourceId source_id_;
  const Vector2<T> init_position_;
  geometry::FrameId ball_frame_id_;
  geometry::GeometryId ball_id_;

  int geometry_id_port_;
  int geometry_pose_port_;
  int state_port_;
  int geometry_query_port_;

  const double diameter_{0.1};  // Ball diameter, just for visualization.
  const double m_{0.1};         // kg
  const double g_{9.81};        // m/s^2
  // Stiffness constant [N/m]. Calculated so that under its own weight the ball
  // penetrates the plane by 1 mm when the contact force and gravitational
  // force are in equilibrium.
  const double k_{m_ * g_ / 0.001};
  // Hunt-Crossley's dissipation factor.
  const double d_{0.0};  // s/m
};

}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake

#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/examples/scene_graph/gen/bouncing_ball_vector.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace scene_graph {
namespace bouncing_ball {

/** A model of a bouncing ball with Hunt-Crossley compliant contact model.
 The model supports 1D motion in a 3D world.

 @tparam_double_only
*/
template <typename T>
class BouncingBallPlant : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BouncingBallPlant)

  /** Constructor
   @param source_id             The source id for this plant to interact with
                                GeoemtrySystem.
   @param scene_graph           Pointer to the geometry system instance on which
                                this plant's geometry will be registered. It
                                must be the same system the source id was
                                extracted from.
   @param p_WB                  The 2D, projected position vector of the ball
                                onto the ground plane relative to the world.
   */
  BouncingBallPlant(geometry::SourceId source_id,
                    geometry::SceneGraph<T>* scene_graph,
                    const Vector2<double>& p_WB);
  ~BouncingBallPlant() override;

  const systems::InputPort<T>& get_geometry_query_input_port() const;
  /** Returns the port to output state. */
  const systems::OutputPort<T>& get_state_output_port() const;
  const systems::OutputPort<T>& get_geometry_pose_output_port() const;

  void set_z(systems::Context<T>* context, const T& z) const {
    get_mutable_state(context).set_z(z);
  }

  void set_zdot(systems::Context<T>* context, const T& zdot) const {
    get_mutable_state(context).set_zdot(zdot);
  }

  /** Mass in kg. */
  double m() const { return m_; }

  /** Stiffness constant in N/m */
  double k() const { return k_; }

  /** Hunt-Crossley's dissipation factor in s/m. */
  double d() const { return d_; }

  /** Gravity in m/s^2. */
  double g() const { return g_; }

 private:
  // Calculate the frame pose set output port value.
  void CalcFramePoseOutput(const systems::Context<T>& context,
                           geometry::FramePoseVector<T>* poses) const;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  static const BouncingBallVector<T>& get_state(
      const systems::ContinuousState<T>& cstate) {
    return dynamic_cast<const BouncingBallVector<T>&>(cstate.get_vector());
  }

  static BouncingBallVector<T>& get_mutable_state(
      systems::ContinuousState<T>* cstate) {
    return dynamic_cast<BouncingBallVector<T>&>(cstate->get_mutable_vector());
  }

  BouncingBallVector<T>* get_mutable_state_output(
      systems::SystemOutput<T>* output) const {
    return dynamic_cast<BouncingBallVector<T>*>(
        output->GetMutableVectorData(state_port_));
  }

  static const BouncingBallVector<T>& get_state(
      const systems::Context<T>& context) {
    return get_state(context.get_continuous_state());
  }

  static BouncingBallVector<T>& get_mutable_state(
      systems::Context<T>* context) {
    return get_mutable_state(&context->get_mutable_continuous_state());
  }

  // The projected position of the ball onto the ground plane. I.e., it's
  // "where the ball bounces".
  const Vector2<double> p_WB_;
  // The id for the ball's frame.
  geometry::FrameId ball_frame_id_;
  // The id for the ball's geometry.
  geometry::GeometryId ball_id_;

  int geometry_pose_port_{-1};
  int state_port_{-1};
  int geometry_query_port_{-1};

  const double diameter_{0.1};  // Ball diameter, just for visualization (m).
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
}  // namespace scene_graph
}  // namespace examples
}  // namespace drake

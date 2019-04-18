#pragma once

#include <memory>

#include <Eigen/Core>

#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/affine_system.h"

namespace drake {
namespace examples {
namespace quadrotor {

/// The Quadrotor - an underactuated aerial vehicle. This version of the
/// Quadrotor is implemented to match the dynamics of the plant specified in
/// the `quadrotor.urdf` model file.
///
/// @system{QuadrotorPlant,
///    @input_port{propellor_force},
///    @output_port{state} @output_port{geometry_pose}
/// }
template <typename T>
class QuadrotorPlant final : public systems::LeafSystem<T> {
 public:
  QuadrotorPlant();
  QuadrotorPlant(double m_arg, double L_arg, const Eigen::Matrix3d& I_arg,
                 double kF_arg, double kM_arg);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit QuadrotorPlant(const QuadrotorPlant<U>&);

  ~QuadrotorPlant() override;

  double m() const { return m_; }
  double g() const { return g_; }

  /// Registers this system as a source for the SceneGraph, adds the
  /// quadrotor geometry, and creates the geometry_pose_output_port for this
  /// system.  This must be called before the a SceneGraph's Context is
  /// allocated.  It can only be called once.
  // TODO(russt): this call only accepts doubles (not T) until SceneGraph
  // supports symbolic.
  void RegisterGeometry(geometry::SceneGraph<double>* scene_graph);

  /// Returns the port to output the pose to SceneGraph.  Users must call
  /// RegisterGeometry() first to enable this port.
  const systems::OutputPort<T>& get_geometry_pose_output_port() const {
    return systems::System<T>::get_output_port(geometry_pose_port_);
  }

  geometry::SourceId source_id() const { return source_id_; }

 private:
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  void CopyStateOut(const systems::Context<T>& context,
                    systems::BasicVector<T>* output) const;

  systems::OutputPortIndex AllocateGeometryPoseOutputPort();

  // Calculator method for the pose output port.
  void CopyPoseOut(const systems::Context<T>& context,
                   geometry::FramePoseVector<T>* poses) const;

  // Allow different specializations to access each other's private data.
  template <typename> friend class QuadrotorPlant;

  // TODO(naveenoid): Declare these as parameters in the context.
  const double g_;           // Gravitational acceleration (m/s^2).
  const double m_;           // Mass of the robot (kg).
  const double L_;           // Length of the arms (m).
  const double kF_;          // Force input constant.
  const double kM_;          // Moment input constant.
  const Eigen::Matrix3d I_;  // Moment of Inertia about the Center of Mass

  // Port handles.
  int state_port_{-1};
  int geometry_pose_port_{-1};

  // Geometry source identifier for this system to interact with SceneGraph.
  geometry::SourceId source_id_{};
  // The id for the quadrotor body.
  geometry::FrameId frame_id_{};
};

/// Generates an LQR controller to move to @p nominal_position. Internally
/// computes the nominal input corresponding to a hover at position @p x0.
/// @see systems::LinearQuadraticRegulator.
std::unique_ptr<systems::AffineSystem<double>> StabilizingLQRController(
    const QuadrotorPlant<double>* quadrotor_plant,
    Eigen::Vector3d nominal_position);

}  // namespace quadrotor
}  // namespace examples

// The following code was added to prevent scalar conversion to symbolic scalar
// types. The QuadrotorPlant makes use of classes that are not compatible with
// the symbolic scalar. This NonSymbolicTraits is explained in
// drake/systems/framework/system_scalar_converter.h.
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<examples::quadrotor::QuadrotorPlant> : public NonSymbolicTraits {
};
}  // namespace scalar_conversion
}  // namespace systems

}  // namespace drake

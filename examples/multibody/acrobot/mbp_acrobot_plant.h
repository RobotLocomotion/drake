#pragma once

#include <limits>
#include <memory>

#include "drake/common/drake_optional.h"
#include "drake/examples/multibody/acrobot/gen/acrobot_state.h"
#include "drake/geometry/geometry_system.h"
#include "drake/multibody/multibody_tree/force_element.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace examples {
namespace multibody {
namespace acrobot {

/// The Acrobot - a canonical underactuated system as described in <a
/// href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter 3
/// of Underactuated Robotics</a>.
/// This plant is modeled using a MultibodyTree.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// @param m1 Mass of link 1 (kg).
/// @param m2 Mass of link 2 (kg).
/// @param l1 Length of link 1 (m).
/// @param l2 Length of link 2 (m).
/// @param lc1 Vertical distance from shoulder joint to center of mass of
/// link 1 (m).
/// @param lc2 Vertical distance from elbow joint to center of mass of
/// link 2 (m).
/// @param Ic1 Inertia of link 1 about the center of mass of link 1
/// (kg*m^2).
/// @param Ic2 Inertia of link 2 about the center of mass of link 2
/// (kg*m^2).
/// @param b1 Damping coefficient of the shoulder joint (kg*m^2/s).
/// @param b2 Damping coefficient of the elbow joint (kg*m^2/s).
/// @param g Gravitational constant (m/s^2).
///
/// The parameters are defaulted to values in Spong's paper (see
/// acrobot_spong_controller.cc for more details).
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd

#if 0
Example using DiagramBuilder::BuildInto:
@code
namespace sample {
class MyDiagram : public Diagram<double> {
 public:
  MyDiagram() {
    DiagramBuilder<double> builder;
    const auto* integrator = builder.AddSystem<Integrator<double>>(1);
    builder.ExportInput(integrator->get_input_port());
    builder.ExportOutput(integrator->get_output_port());
    builder.BuildInto(this);
  }
};
@endcode
#endif

template<typename T>
class MbpAcrobotPlant final : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MbpAcrobotPlant)

  /// Constructor for an acrobot model with default parameters. See this plant's
  /// documentation for details on the parameters for the plant.
  /// @warning Damping is still not included in this model.
  // TODO(amcastro-tri): Add damping.
  MbpAcrobotPlant(double m1 = 1.0,
               double m2 = 1.0,
               double l1 = 1.0,
               double l2 = 2.0,
               double lc1 = 0.5,
               double lc2 = 1.0,
               double Ic1 = .083,
               double Ic2 = .33,
               double b1 = 0.1,
               double b2 = 0.1,
               double g = 9.81);

  /// Constructor for an acrobot model with parameters specified in the default
  /// constructor.
  /// This constructor registers `this` plant as a source for `geometry_system`
  /// as well as the frames and geometry used for visualization.
  explicit MbpAcrobotPlant(geometry::GeometrySystem<double>* geometry_system);

  /// Scalar-converting copy constructor.
  //template <typename U>
  //explicit MbpAcrobotPlant(const MbpAcrobotPlant<U>&);

  // getters for robot parameters
  double m1() const { return m1_; }
  double m2() const { return m2_; }
  double l1() const { return l1_; }
  double l2() const { return l2_; }
  double lc1() const { return lc1_; }
  double lc2() const { return lc2_; }
  double Ic1() const { return Ic1_; }
  double Ic2() const { return Ic2_; }
  double b1() const { return b1_; }
  double b2() const { return b2_; }
  double g() const { return g_; }

  // getters for joints:
  const drake::multibody::RevoluteJoint<T>& shoulder() const {
    DRAKE_DEMAND(shoulder_ != nullptr);
    return *shoulder_;
  }
  const drake::multibody::RevoluteJoint<T>& elbow() const {
    DRAKE_DEMAND(elbow_ != nullptr);
    return *elbow_;
  }

#if 0
  /// Evaluates the input port and returns the scalar value
  /// of the commanded torque on the elbow joint.
  const T& get_tau(const systems::Context<T>& context) const {
    const Eigen::VectorBlock<const VectorX<T>> input =
        this->template EvalEigenVectorInput(context, applied_torque_input_);
    DRAKE_DEMAND(input.size() == 1);
    return input.coeff(0);
  }
#endif

#if 0
  /// Returns the unique id identifying this plant as a source for a
  /// GeometrySystem.
  /// Returns `nullopt` if `this` plant did not register any geometry.
  optional<geometry::SourceId> get_source_id() const {
    return source_id_;
  }

  /// Returns the output port of frame id's used to communicate poses to a
  /// GeometrySystem. It throws a std::out_of_range exception if this system was
  /// not registered with a GeometrySystem.
  const systems::OutputPort<T>& get_geometry_id_output_port() const;

  /// Returns the output port of frames' poses to communicate with a
  /// GeometrySystem. It throws a std::out_of_range exception if this system was
  /// not registered with a GeometrySystem.
  const systems::OutputPort<T>& get_geometry_pose_output_port() const;

  const systems::InputPortDescriptor<T>& get_input_port() const;

  const systems::OutputPort<T>& get_state_output_port() const;
#endif

#if 0
  /// Sets the state in `context` so that generalized positions and velocities
  /// are zero.
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override {
    DRAKE_DEMAND(state != nullptr);
    model_->SetDefaultState(context, state);
  }
#endif

 private:
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class MbpAcrobotPlant;

  // Helper method for NaN initialization.
  static constexpr T nan() {
    return std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN();
  }

  // Helper method to build the MultibodyTree model of the system.
  //void BuildMultibodyTreeModel();

  std::unique_ptr<::drake::multibody::multibody_plant::MultibodyPlant<T>>
  MakeMultibodyPlant() const;

  void RegisterGeometry(geometry::GeometrySystem<double>* geometry_system);

  // Copies the state in `context` to `output`.
  //void CopyStateOut(const systems::Context<T>& context,
    //                AcrobotState<T>* output) const;

  // The physical parameters of the model. They are initialized with NaN for a
  // quick detection of uninitialized values.
  double m1_{1.0}, m2_{1.0},  // In kilograms.
      l1_{1.0}, l2_{2.0},     // In meters.
      lc1_{0.5}, lc2_{1.0},   // In meters.
      Ic1_{.083}, Ic2_{.33},   // In Kgr⋅m².
      b1_{0.1}, b2_{0.1},     // In N⋅m⋅s.
      g_{9.81};                  // In m/s².

  // The entire multibody model.
  std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<T>> model_;

  // Bodies:
  const drake::multibody::RigidBody<T>* link1_;  // Upper link.
  const drake::multibody::RigidBody<T>* link2_;  // Lower link.
  // Joints:
  const drake::multibody::RevoluteJoint<T>* shoulder_;
  const drake::multibody::RevoluteJoint<T>* elbow_;

  // Geometry source identifier for this system to interact with geometry
  // system. It is made optional for plants that do not register geometry
  // (dynamics only).
  optional<geometry::SourceId> source_id_{nullopt};

  // Frame Id's for each body in the model:
  optional<geometry::FrameId> link1_frame_id_{nullopt};
  optional<geometry::FrameId> link2_frame_id_{nullopt};

  // Port handles
  int geometry_id_port_{-1};
  int geometry_pose_port_{-1};
  int applied_torque_input_{-1};
  int state_output_port_{-1};
};

}  // namespace acrobot
}  // namespace multibody
}  // namespace examples
}  // namespace drake

// Disable support for symbolic evaluation.
// TODO(amcastro-tri): Allow symbolic evaluation once MultibodyTree supports it.
namespace drake {
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<drake::examples::multibody::acrobot::MbpAcrobotPlant> :
    public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake

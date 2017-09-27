#pragma once

#include <memory>

#include "drake/geometry/geometry_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

namespace drake {
namespace examples {
namespace n_link_pendulum {

/// The Acrobot - a canonical underactuated system as described in <a
/// href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter 3
/// of Underactuated Robotics</a>.
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
/// acrobot_spong_controller.cc for more details). Alternatively, an instance
/// of AcrobotMultibodyPlant using parameters of MIT lab's acrobot can be created by
/// calling the static method CreateAcrobotMIT();
///
/// Note that the Spong controller behaves differently on these two sets of
/// parameters. The controller works well on the first set of parameters,
/// which Spong used in his paper. In contrast, it is difficult to find a
/// functional set of gains to stabilize the robot about its upright fixed
/// point using the second set of parameters, which represent a real robot.
/// This difference illustrates limitations of the Spong controller.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
template<typename T>
class NLinkPendulumPlant : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NLinkPendulumPlant)

  NLinkPendulumPlant(double mass, double length, double radius, int num_links,
                     geometry::GeometrySystem<T>* geometry_system);

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit NLinkPendulumPlant(const NLinkPendulumPlant<U>&);

  double get_mass() const { return mass_; }

  double get_length() const { return length_; }

  double get_radius() const { return radius_; }

  double get_num_links() const { return num_links_; }

  void SetDefaultState(const systems::Context<T>&,
                       systems::State<T>*) const override;

 protected:
  // No inputs implies no feedthrough; this makes it explicit.
  optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return false;
  }

 protected:
  T DoCalcKineticEnergy(const systems::Context<T>& context) const override;
  T DoCalcPotentialEnergy(const systems::Context<T>& context) const override;

 private:
  // Override of context construction so that we can delegate it to
  // MultibodyTree.
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // Helper method to build the MultibodyTree model of the system.
  void BuildMultibodyTreeModel(geometry::GeometrySystem<T>* geometry_system);

  // Allocate the id output.
  geometry::FrameIdVector AllocateFrameIdOutput(
      const systems::Context<T>& context) const;
  // Calculate the id output.
  void CalcFrameIdOutput(
      const systems::Context<T>& context,
      geometry::FrameIdVector* id_set) const;

  double mass_;
  double length_;
  double radius_;
  int num_links_;
  multibody::MultibodyTree<T> model_;
  std::vector<const multibody::RevoluteJoint<T>*> joints_;

  // Geometry source identifier for this system to interact with geometry system
  geometry::SourceId source_id_{};
  // The id in GeometrySystem for each link's frame.
  std::vector<geometry::FrameId> body_ids_;

  // Port handles
  int geometry_id_port_{-1};
  int geometry_pose_port_{-1};
};

}  // namespace n_link_pendulum
}  // namespace examples
}  // namespace drake

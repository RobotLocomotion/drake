#pragma once

#include <memory>

#include "drake/geometry/geometry_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

namespace drake {
namespace examples {
namespace n_link_pendulum {

template<typename T>
class NLinkPendulumPlant final : public multibody::MultibodyPlant<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NLinkPendulumPlant)

  NLinkPendulumPlant(double mass, double length, double radius, int num_links,
                     geometry::GeometrySystem<T>* geometry_system);

  /// Scalar-converting copy constructor.
// TODO: read scalar conversion's doc on how to do this for non-final systems
// like MultibodyPlant.
#if 0
  template <typename U>
  explicit NLinkPendulumPlant(const NLinkPendulumPlant<U>&);
#endif

  double get_mass() const { return mass_; }

  double get_length() const { return length_; }

  double get_radius() const { return radius_; }

  double get_num_links() const { return num_links_; }

  void SetStraightAtAnAngle(systems::Context<T>*, const T& angle) const;

 private:
  void BuildMultibodyModel(multibody::MultibodyTree<T>* model) override;

  void DoRegisterGeometry(
      geometry::GeometrySystem<T>* geometry_system) const override;

  double mass_;
  double length_;
  double radius_;
  int num_links_;
  std::vector<const multibody::RevoluteJoint<T>*> joints_;
};

}  // namespace n_link_pendulum
}  // namespace examples
}  // namespace drake

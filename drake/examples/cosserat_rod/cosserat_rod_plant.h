#pragma once

#include <memory>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/multibody_tree/modeler/multibody_modeler.h"

namespace drake {
namespace examples {
namespace cosserat_rod {

///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
template <typename T>
class CosseratRodPlant : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CosseratRodPlant)

  /// Constructs a Cosserat model for a rod of circular cross section.
  CosseratRodPlant(double length, double radius, double mass,
                   double young_modulus, double shear_modulus,
                   double tau_bending, double tau_twisting,
                   int num_links);

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit CosseratRodPlant(const CosseratRodPlant<U>&);

  const multibody::MultibodyModeler<T>& get_modeler() const {
    return modeler_;
  }

  double mass() const { return mass_; }

 protected:
  T DoCalcKineticEnergy(const systems::Context<T>& context) const override;
  T DoCalcPotentialEnergy(const systems::Context<T>& context) const override;

 private:
  // Override of context construction so that we can delegate it to
  // MultibodyModeler.
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

  void OutputState(const systems::Context<T>& context,
                   systems::BasicVector<T>* state_port_value) const;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // Helper method to create a link segment and add it to the modeler.
  const multibody::RigidLink<T>& AddLinkElement(
      int element_index, const multibody::Link<T>& parent, const T& s);

  void BuildMultibodyModeler();

  // Geometry parameters:
  double length_{0.0};

  // Damping coefficient.
  double mass_{0.0};
  double tau_bending_{0.0};
  double tau_twisting_{0.0};

  // Numerical parameters:
  int num_elements_{0};

  // Functors to compute cross section properties as a function of arc length.
  // A(s) = area_(s).
  std::function<T(const T&)> area_;
  // Momemnts of inertia, Ii(s) = moment_of_inertiai_(s):
  std::function<T(const T&)> moment_of_inertia1_;
  std::function<T(const T&)> moment_of_inertia2_;
  std::function<T(const T&)> moment_of_inertia3_;

  // Material parameters functors:
  // E(s) = young_modulus_
  std::function<T(const T&)> young_modulus_;

  // G(s) = shear_modulus_(s)
  std::function<T(const T&)> shear_modulus_;


  // Output ports:
  systems::OutputPortIndex state_output_port_index_;
  systems::OutputPortIndex energy_output_port_index_;

  multibody::MultibodyModeler<T> modeler_;
  //const multibody::Link<T>* link1_{nullptr};
  const multibody::Link<T>* link2_{nullptr};
  //const multibody::RevoluteJoint<T>* elbow_{nullptr};
};

}  // namespace cosserat_rod
}  // namespace examples
}  // namespace drake

// Disable scalar conversion from/to symbolic::Expresion.
namespace drake {
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<examples::cosserat_rod::CosseratRodPlant> :
    public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake

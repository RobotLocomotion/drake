#pragma once

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace box {

/// A model of the spring interaction force between
///   two boxes. The output is the force to be applied
///   on box 2; box one should receive the opposite.
/// Box 1's state is passed into first_box_input
/// Box 2's state is passed into second_box_input.
/// @f[ u_2 = -k (q2 - q1) @f]
///
/// @system{SpringPlant,
///    @input_port{q1, q-dot1},
///    @input_port{q2, q-dot2},
///    @output_port{u2}
/// }
///
/// @params: stiffness (k), damping (d), rest length (l),
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
template <typename T>
class SpringPlant final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpringPlant);

  /// Constructs a default plant.
  SpringPlant();

  /// constructs a plant with stiffness (k), damping (d), rest length (l),
  SpringPlant(double k, double d, double l);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit SpringPlant(const SpringPlant<U>&);

  ~SpringPlant() final;

  /// Returns the port to output force.
  const systems::OutputPort<T>& get_force_output_port() const;
  const systems::InputPort<T>& get_first_box_input_port() const;
  const systems::InputPort<T>& get_second_box_input_port() const;
  

  /// Calculates the kinetic + potential energy.
  T CalcTotalEnergy(const systems::Context<T>& context) const;

  /// Evaluates the input port and returns the scalar value
  /// of the commanded torque.
  T get_u2(const systems::Context<T>& context) const
  {
    return this->get_force_output_port().Eval(context)(0);
  }

 private:
   // SpringPlant of one scalar type is friends with all other scalar types.
  template <typename>
  friend class SpringPlant;
  void CalcVectorOutput(
      const systems::Context<T>& context,
      systems::BasicVector<T>* output) const;
  
  double k_ = {1.0}; /* stiffness */
  double d_ = {0.0}; /* damping */
  double l_ = {1.0}; /* rest length */
};

}  // namespace box
}  // namespace examples
}  // namespace drake

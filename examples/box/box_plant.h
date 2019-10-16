#pragma once

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {
namespace examples {
namespace box {

/// A model of a simple box, where q is the center point. This box
/// is assumed to rest on a plane with normal force f_n and
/// Stribeck friction in the q direction.
///
/// @f[ m \ddot q = u @f]
///
/// @system{BoxPlant,
///    @input_port{u},
///    @output_port{q}
/// }
///
/// @params: inverse mass (i_m), length (l), velocity damping (d), 
///          normal force (f_n), friction coefficients (mu_s, mu_k),
///          stiction velocity tolerance (v_s)
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
template <typename T>
class BoxPlant final : public systems::VectorSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BoxPlant);

  /// Constructs a default plant.
  BoxPlant();

  /// Constructs a plant with inv mass i_m, length l, and damping d
  BoxPlant(double i_m, double l, double d);

  /// Constructs a plant with inv mass i_m, length l, damping d,
  ///   normal force f_n, friction coeffs mu_s and mu_k, and stiction
  ///   velocity v_s
  BoxPlant(double i_m, double l, double d, double f_n,
    double mu_s, double mu_k, double v_s);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit BoxPlant(const BoxPlant<U>&);

  ~BoxPlant() final;

  /// Returns the port to output state.
  const systems::OutputPort<T>& get_state_output_port() const
  {
    return this->get_output_port();
  }

  /// Calculates the kinetic energy.
  T CalcTotalEnergy(const systems::Context<T>& context) const;

  T CalcFrictionFromVelocity(T velocity) const;
  
  T CalcFriction(const systems::Context<T>& context) const;

  /// Evaluates the input port and returns the scalar value
  /// of the commanded torque.
  T get_u(const systems::Context<T>& context) const {
    return this->get_input_port().Eval(context)(0);
  }

  double CalcIterationLimiterAlpha(const VectorX<T>& x0, const VectorX<T>& dx) const;

  double get_length() const { return l_; }

  double get_inv_mass() const { return i_m_; }

  double get_f_n() const { return f_n_; }

  double get_mu_s() const { return mu_s_; }

  double get_mu_k() const { return mu_k_; }

  double get_v_s() const { return v_s_; }

  Eigen::VectorBlock<const VectorX<T>> GetBoxState(const systems::Context<T>& context) const {
    return this->GetVectorState(context);
  }

  static void set_initial_state(systems::Context<T>* context,
                                const Eigen::Ref< const VectorX<T> >& z0);

  void StoreEigenCSVwFriction(const std::string& filename, const VectorX<double>& times, const MatrixX<double>& data, const VectorX<double>& input_vector) const;


 private:

  // BoxPlant of one scalar type is friends with all other scalar types.
  template <typename>
  friend class BoxPlant;
  
  void DoCalcVectorOutput(
      const systems::Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const final;

  void DoCalcVectorTimeDerivatives(const systems::Context< T > &context, 
      const Eigen::VectorBlock< const VectorX< T >> &input, 
      const Eigen::VectorBlock< const VectorX< T >> &state, 
      Eigen::VectorBlock< VectorX< T >> *derivatives) const final;

  double i_m_;
  double l_;
  double d_;
  double f_n_;
  double mu_s_;
  double mu_k_;
  double v_s_;
};

}  // namespace box
}  // namespace examples
}  // namespace drake

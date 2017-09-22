#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/vector_system.h"
#include "drake/systems/primitives/affine_system.h"

namespace drake {
namespace systems {

/**
 * Interface class for a discrete- or continuous-time, piecewise-affine
 * system.
 *
 * If `time_period > 0.0`, then the system will be discrete-time with the state
 * update:
 *   @f[ x(t+h) = A_i x(t) + B_i u(t) + f_{0,i}, \forall (x_i,u_i) \in P_i, @f]
 * where `u` denotes the input vector, `x` denotes the state vector, `h` is the
 * time_period,
 * and `P_i` represents the domain of the `i`th piece.
 * If `time_period == 0.0`, then the system will be continuous-time with the
 * time derivatives:
 *   @f[ \dot{x}(t) = A_i x(t) + B_i u(t) + f_{0,i}, \forall (x_i,u_i) \in P_i.
 * @f]
 *
 * In both cases, the system will have the output:
 *   @f[ y(t) = C x(t) + D u(t) + y_0, \forall (x_i,u_i) \in P_i, @f]
 * where `y` denotes the output vector.
 *
 * Note that this interface class does not place any restrictions on the form of
 * $P_i$ -- derived classes
 * must define it by implementing the function %DoGetSystemIndexAtContext%.
 * Derived classes must also implement
 * %get_num_pieces% and %GetSystemAtIndex%.
 *
 * @tparam T The scalar element type, which must be a valid Eigen scalar.
 */
template <typename T>
class PiecewiseAffineSystem : public VectorSystem<T> {
  // TODO(russt): systems::AffineSystem<T> is only being used here to wrap the
  // A,B,...,y0 matrices.  Consider
  // just making a local structure here if constructing/carrying around systems
  // because too heavy.

 public:
  /// Defines the total number of pieces (e.g. the maximum index+1) that
  /// describes the system.
  virtual int get_num_pieces() const = 0;

  /// Returns the state-space description of the affine dynamics in `P_{index}`.
  /// Indices are numbered
  /// 0...{num_pieces-1}.  Derived classes should implement
  /// %DoGetSystemAtIndex%.
  const AffineSystem<T>& GetSystemAtIndex(int index) const {
    DRAKE_DEMAND(index >= 0);
    DRAKE_ASSERT(index < get_num_pieces());
    return DoGetSystemAtIndex(index);
  }

  /// Returns the index `i` at the given context, implicitly defining `P_i`.
  /// Derived classes should
  /// implement %DoGetSystemIndexAtContext%.
  int GetSystemIndexAtContext(const Context<T>& context) const {
    const auto& input = this->EvalVectorInput(context);
    const auto& state = this->GetVectorState(context);
    int index = DoGetSystemIndexAtContext(context, input, state);
    DRAKE_DEMAND(index >= 0);
    DRAKE_ASSERT(index < get_num_pieces());
    return index;
  }
  /// Returns the state-space description of the dynamics at the current
  /// %Context%.
  const AffineSystem<T>& GetSystemAtContext(const Context<T>& context) const {
    const auto& input = this->EvalVectorInput(context);
    const auto& state = this->GetVectorState(context);
    return DoGetSystemAtContext(context, input, state);
  }

  // Access methods.
  double time_period() const { return time_period_; }
  int num_states() const { return num_states_; }
  int num_inputs() const { return num_inputs_; }
  int num_outputs() const { return num_outputs_; }

 protected:
  /// Constructs a new piecewise-affine system where all pieces have the same
  /// input, state, and output
  /// dimensions.  The system will be continuous-time if `time_period == 0.0`,
  /// or discrete-time if `time_period > 0.0`.
  PiecewiseAffineSystem(int input_size, int state_size, int output_size,
                        double time_period = 0.0);

  /// Derived classes must implement this.  Returns the affine coefficients
  /// describing the dynamics in piece `i`.
  virtual const AffineSystem<T>& DoGetSystemAtIndex(int index) const = 0;

  /// Derived classes must implement this.  Indices which piece is active at the
  /// current %Context%.  The returned index
  /// must be within the range `[0, get_num_pieces() )`.
  virtual int DoGetSystemIndexAtContext(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state) const = 0;

  /// Convenience method for retrieving the system coefficients directly from
  /// the current %Context%.  Derived classes
  /// may choose to implement this directly if it is more efficient or more
  /// intuitive than going through an explicit
  /// index number.
  const AffineSystem<T>& DoGetSystemAtContext(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state) const {
    return GetSystemAtIndex(DoGetSystemIndexAtContext(context, input, state));
  }

 private:
  // VectorSystem overrides.
  void DoCalcVectorOutput(const Context<T>& context,
                          const Eigen::VectorBlock<const VectorX<T>>& input,
                          const Eigen::VectorBlock<const VectorX<T>>& state,
                          Eigen::VectorBlock<VectorX<T>>* output) const;

  void DoCalcVectorTimeDerivatives(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* derivatives) const;

  void DoCalcVectorDiscreteVariableUpdates(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* next_state) const;

  const int num_states_{0};
  const int num_inputs_{0};
  const int num_outputs_{0};
  const double time_period_{0.0};
};

/**
 * Interface class for a discrete- or continuous-time, piecewise-affine system
 * defined over convex polytopic domains.
 *
 * This defines additional interface methods to return the domains in their
 * half-space representation:
 *  @f[ P_i := \{ x,u | P_x x + P_u u \le p_{rhs} \}, @f]
 *
 * @tparam T The scalar element type, which must be a valid Eigen scalar.
 */
template <typename T>
class PolytopicPiecewiseAffineSystem : public PiecewiseAffineSystem<T> {
 public:
  /// Describes the input/state domain of a piece `P` using the half-space
  /// representation (aka. H-rep):
  ///   Px*x + Pu*u <= prhs.
  class Domain {
   public:
    Domain(const Eigen::Ref<const Eigen::MatrixXd>& Px,
           const Eigen::Ref<const Eigen::MatrixXd>& Pu,
           const Eigen::Ref<const Eigen::VectorXd>& prhs)
        : Px_(Px), Pu_(Pu), prhs_(prhs) {
      DRAKE_DEMAND(Px.rows() == Pu.rows());
      DRAKE_DEMAND(Px.rows() == prhs.rows());
    }
    const Eigen::MatrixXd& Px() const { return Px_; }
    const Eigen::MatrixXd& Pu() const { return Pu_; }
    const Eigen::VectorXd& prhs() const { return prhs_; }

   private:
    const Eigen::MatrixXd Px_;
    const Eigen::MatrixXd Pu_;
    const Eigen::VectorXd prhs_;
  };

  /// Returns the %Domain% of the `i`the piece of the piecewise-affine system.
  const Domain& GetDomainAtIndex(int index) const {
    DRAKE_DEMAND(index >= 0);
    DRAKE_ASSERT(index < this->get_num_pieces());
    return DoGetDomainAtIndex(index);
  }

  /// Returns the %Domain% of the affine system governing the dynamics at the
  /// current %Context%.
  virtual const Domain& GetDomainAtContext(const Context<T>& context) const {
    const auto& input = this->EvalVectorInput(context);
    const auto& state = this->GetVectorState(context);
    const Domain& d = DoGetDomainAtContext(context, input, state);
    DRAKE_ASSERT(
        ((d.Pu() * input + d.Px() * state).array() <= d.prhs().array()).all());
    return d;
  }

 protected:
  /// Constructs the system, passing arguments to the %PiecewiseAffineSystem%
  /// constructor.
  PolytopicPiecewiseAffineSystem(int input_size, int state_size,
                                 int output_size, double time_period = 0.0)
      : PiecewiseAffineSystem<T>(input_size, state_size, output_size,
                                 time_period) {}

  /// Derived classes should implement this method to define the %Domain% of
  /// `i`th piece.
  virtual const Domain& DoGetDomainAtIndex(int index) const = 0;

  /// Provides a direct method for finding the %Domain% at the current
  /// %Context%. Derived classes
  /// may choose to implement this directly if it is more efficient or more
  /// intuitive than going through an explicit
  /// index number.
  virtual const Domain& DoGetDomainAtContext(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state) const {
    return GetDomainAtIndex(
        this->DoGetSystemIndexAtContext(context, input, state));
  }
};

/**
 * Provides a naive implementation of the %PolytopicPiecewiseAffineSystem%
 * interface, by simply maintaining a
 * list of polytopic pieces, and doing a linear search through the list of
 * domains to perform the lookups.
 *
 * @tparam T The scalar element type, which must be a valid Eigen scalar.
 */
template <typename T>
class PolytopicListPiecewiseAffineSystem
    : public PolytopicPiecewiseAffineSystem<T> {
 public:
  /// Constructs the system, passing arguments to the %PiecewiseAffineSystem%
  /// constructor.  However, most system methods
  /// will abort with failure if called before at least one piece has been added
  /// using the %AddPiece% method.
  PolytopicListPiecewiseAffineSystem(int input_size, int state_size,
                                     int output_size, double time_period = 0.0)
      : PolytopicPiecewiseAffineSystem<T>(input_size, state_size, output_size,
                                          time_period) {}

  using Domain = typename PolytopicPiecewiseAffineSystem<T>::Domain;

  /// Declares that the dynamics of the system over the %Domain% are governed by
  /// the %AffineSystem%.
  ///
  /// In the case that this %Domain% has a non-empty intersection with the
  /// %Domain% of another piece, the piece with
  /// the smaller index will be returned during %Context%-based queries.
  ///
  /// \return the index of the newly added piece.
  ///
  // TODO(russt): Throw if the domain to be added overlaps with an existing
  // domain.
  int AddPiece(std::unique_ptr<AffineSystem<T>> system,
               std::unique_ptr<Domain> domain);

  /// Declares that the dynamics and output of the system over the domain { x,u
  /// | P_x x * P_u u <= p_rhs } using the
  /// standard %AffineSystem% coefficients.
  ///
  /// In the case that this %Domain% has a non-empty intersection with the
  /// %Domain% of another piece, the piece with
  /// the smaller index will be returned during %Context%-based queries.
  ///
  /// \return the index of the newly added piece.
  ///
  int AddPiece(const Eigen::Ref<const Eigen::MatrixXd>& A,
               const Eigen::Ref<const Eigen::MatrixXd>& B,
               const Eigen::Ref<const Eigen::VectorXd>& f0,
               const Eigen::Ref<const Eigen::MatrixXd>& C,
               const Eigen::Ref<const Eigen::MatrixXd>& D,
               const Eigen::Ref<const Eigen::VectorXd>& y0,
               const Eigen::Ref<const Eigen::MatrixXd>& Px,
               const Eigen::Ref<const Eigen::MatrixXd>& Pu,
               const Eigen::Ref<const Eigen::VectorXd>& prhs);

  /// Returns the total number of pieces that have been added.
  int get_num_pieces(void) const { return systems_.size(); }

 protected:
  // PolytopicPiecewiseAffineSystem overloads.

  const AffineSystem<T>& DoGetSystemAtIndex(int index) const {
    DRAKE_DEMAND(index >= 0 && index <= static_cast<int>(systems_.size()));
    return *(systems_[index]);
  }

  const Domain& DoGetDomainAtIndex(int index) const {
    DRAKE_DEMAND(index >= 0 && index <= static_cast<int>(domains_.size()));
    return *(domains_[index]);
  }

  int DoGetSystemIndexAtContext(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state) const;

 private:
  std::vector<std::unique_ptr<AffineSystem<T>>> systems_{};
  std::vector<std::unique_ptr<Domain>> domains_{};
};

}  // namespace systems
}  // namespace drake

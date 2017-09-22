#include "drake/systems/primitives/piecewise_affine_system.h"

#include <memory>
#include <utility>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

template <typename T>
PiecewiseAffineSystem<T>::PiecewiseAffineSystem(int input_size, int state_size,
                                                int output_size,
                                                double time_period)
    : VectorSystem<T>(input_size, output_size),
      num_states_(state_size),
      num_inputs_(input_size),
      num_outputs_(output_size),
      time_period_(time_period) {
  DRAKE_DEMAND(num_states_ >= 0);
  DRAKE_DEMAND(num_inputs_ >= 0);
  DRAKE_DEMAND(num_outputs_ >= 0);
  DRAKE_DEMAND(time_period_ >= 0.0);

  if (num_states_ > 0) {
    if (time_period_ == 0.0) {
      this->DeclareContinuousState(num_states_);
    } else {
      this->DeclareDiscreteState(num_states_);
      this->DeclarePeriodicDiscreteUpdate(time_period_, 0.0);
    }
  }
}

template <typename T>
void PiecewiseAffineSystem<T>::DoCalcVectorOutput(
    const Context<T>& context,
    const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* output) const {
  const AffineSystem<T>& system = DoGetSystemAtContext(context, input, state);
  *output = system.C() * state + system.y0();
  if (this->num_inputs_ > 0) {
    *output += system.D() * input;
  }
}

template <typename T>
void PiecewiseAffineSystem<T>::DoCalcVectorTimeDerivatives(
    const Context<T>& context,
    const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* derivatives) const {
  const AffineSystem<T>& system = DoGetSystemAtContext(context, input, state);
  *derivatives = system.A() * state + system.f0();
  if (this->num_inputs_ > 0) {
    *derivatives += system.B() * input;
  }
}

template <typename T>
void PiecewiseAffineSystem<T>::DoCalcVectorDiscreteVariableUpdates(
    const Context<T>& context,
    const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* next_state) const {
  const AffineSystem<T>& system = DoGetSystemAtContext(context, input, state);
  *next_state = system.A() * state + system.f0();
  if (this->num_inputs_ > 0) {
    *next_state += system.B() * input;
  }
}

template <typename T>
// Returns the index of the newly added piece.
int PolytopicListPiecewiseAffineSystem<T>::AddPiece(
    std::unique_ptr<AffineSystem<T>> system,
    std::unique_ptr<Domain> domain) {
  DRAKE_DEMAND(system->num_states() == this->num_states());
  DRAKE_DEMAND(system->num_inputs() == this->num_inputs());
  DRAKE_DEMAND(system->num_outputs() == this->num_outputs());
  DRAKE_DEMAND(system->time_period() == this->time_period());
  DRAKE_DEMAND(domain->Px().cols() == this->num_states());
  DRAKE_DEMAND(domain->Pu().cols() == this->num_inputs());

  systems_.push_back(std::move(system));
  domains_.push_back(std::move(domain));
  return systems_.size() - 1;
}

template <typename T>
int PolytopicListPiecewiseAffineSystem<T>::AddPiece(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::VectorXd>& f0,
    const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::MatrixXd>& D,
    const Eigen::Ref<const Eigen::VectorXd>& y0,
    const Eigen::Ref<const Eigen::MatrixXd>& Px,
    const Eigen::Ref<const Eigen::MatrixXd>& Pu,
    const Eigen::Ref<const Eigen::VectorXd>& prhs) {
  // construct the affine system + domain and pass it to the method above
  DRAKE_DEMAND(A.rows() == this->num_states());
  DRAKE_DEMAND(B.cols() == this->num_inputs());
  DRAKE_DEMAND(C.rows() == this->num_outputs());
  DRAKE_DEMAND(Px.cols() == this->num_states());
  DRAKE_DEMAND(Pu.cols() == this->num_inputs());

  systems_.push_back(std::make_unique<AffineSystem<T>>(A, B, f0, C, D, y0,
                                                       this->time_period()));
  domains_.push_back(std::make_unique<Domain>(Px, Pu, prhs));
  return systems_.size() - 1;
}

template <typename T>
int PolytopicListPiecewiseAffineSystem<T>::DoGetSystemIndexAtContext(
    const Context<T>& context,
    const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state) const {
  unused(context);
  // linear search through domain_list, and return the first match.
  // assert if you get to the end.
  for (int i = 0; i < static_cast<int>(domains_.size()); i++) {
    if (((domains_[i]->Px() * state + domains_[i]->Pu() * input).array() <=
         (domains_[i]->prhs()).array())
            .all())
      return i;
  }
  throw std::runtime_error("No system defined for this context.");
}

template class PolytopicListPiecewiseAffineSystem<double>;

}  // namespace systems
}  // namespace drake

#include "drake/systems/sensors/rotary_encoders.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace sensors {

template <typename T>
RotaryEncoders<T>::RotaryEncoders(const std::vector<int>& ticks_per_revolution)
    : num_encoders_(ticks_per_revolution.size()),
      ticks_per_revolution_(ticks_per_revolution) {
  DRAKE_ASSERT(*std::min_element(ticks_per_revolution_.begin(),
                                 ticks_per_revolution_.end()) >= 0);
  this->DeclareInputPort(systems::kVectorValued, num_encoders_);
  this->DeclareOutputPort(systems::kVectorValued, num_encoders_);
}

template <typename T>
RotaryEncoders<T>::RotaryEncoders(int input_port_size,
                                  const std::vector<int>& input_vector_indices)
    : num_encoders_(input_vector_indices.size()),
      indices_(input_vector_indices) {
  DRAKE_DEMAND(input_port_size >= 0);
  DRAKE_ASSERT(*std::min_element(indices_.begin(), indices_.end()) >= 0);
  DRAKE_ASSERT(*std::max_element(indices_.begin(), indices_.end()) <
               input_port_size);
  this->DeclareInputPort(systems::kVectorValued, input_port_size);
  this->DeclareOutputPort(systems::kVectorValued, num_encoders_);
}

template <typename T>
RotaryEncoders<T>::RotaryEncoders(int input_port_size,
                                  const std::vector<int>& input_vector_indices,
                                  const std::vector<int>& ticks_per_revolution)
    : num_encoders_(input_vector_indices.size()),
      indices_(input_vector_indices),
      ticks_per_revolution_(ticks_per_revolution) {
  DRAKE_DEMAND(input_port_size >= 0);
  DRAKE_ASSERT(*std::min_element(indices_.begin(), indices_.end()) >= 0);
  DRAKE_ASSERT(*std::max_element(indices_.begin(), indices_.end()) <
               input_port_size);
  DRAKE_DEMAND(ticks_per_revolution_.empty() ||
               indices_.size() == ticks_per_revolution_.size());
  DRAKE_ASSERT(ticks_per_revolution_.empty() ||
               *std::min_element(ticks_per_revolution_.begin(),
                                 ticks_per_revolution_.end()) >= 0);
  this->DeclareInputPort(systems::kVectorValued, input_port_size);
  this->DeclareOutputPort(systems::kVectorValued, num_encoders_);
}

template <typename T>
void RotaryEncoders<T>::DoCalcOutput(const systems::Context<T>& context,
                                     systems::SystemOutput<T>* output) const {
  Eigen::VectorBlock<VectorX<T>> y =
      output->GetMutableVectorData(0)->get_mutable_value();
  const Eigen::VectorBlock<const VectorX<T>>& calibration_offsets =
      this->GetNumericParameter(context, 0).get_value();
  DRAKE_ASSERT(calibration_offsets.size() == num_encoders_);
  const BasicVector<T>* input = this->EvalVectorInput(context, 0);

  // Loop through the outputs.
  for (int i = 0; i < num_encoders_; i++) {
    const int index = indices_.empty() ? i : indices_[i];

    // Calibration.
    y(i) = input->GetAtIndex(index) - calibration_offsets(i);

    // Quantization.
    if (!ticks_per_revolution_.empty()) {
      using std::floor;
      y(i) = floor(y(i) * ticks_per_revolution_[i] / M_2_PI) * M_2_PI /
             ticks_per_revolution_[i];
    }
  }
}

template <typename T>
std::unique_ptr<systems::Parameters<T>> RotaryEncoders<T>::AllocateParameters()
    const {
  // Use parameters for the (unnamed) calibration offsets.
  return std::make_unique<systems::Parameters<T>>(
      std::make_unique<systems::BasicVector<T>>(num_encoders_));
}

template <typename T>
void RotaryEncoders<T>::SetDefaultParameters(
    const systems::LeafContext<T>& context,
    systems::Parameters<T>* params) const {
  params->get_mutable_numeric_parameter(0)->SetZero();
}

template <typename T>
void RotaryEncoders<T>::set_calibration_offsets(
    systems::Context<T>* context,
    const Eigen::Ref<VectorX<T>>& calibration_offsets) const {
  auto leaf_context = dynamic_cast<systems::LeafContext<T>*>(context);
  DRAKE_DEMAND(leaf_context != nullptr);
  DRAKE_DEMAND(calibration_offsets.rows() == num_encoders_);
  leaf_context->set_parameters(std::make_unique<systems::Parameters<T>>(
      std::make_unique<systems::BasicVector<T>>(calibration_offsets)));
}

template <typename T>
Eigen::VectorBlock<const VectorX<T>> RotaryEncoders<T>::get_calibration_offsets(
    const systems::Context<T>& context) const {
  return this->template GetNumericParameter(context, 0).get_value();
}

template <typename T>
RotaryEncoders<AutoDiffXd>* RotaryEncoders<T>::DoToAutoDiffXd() const {
  return new RotaryEncoders<AutoDiffXd>(this->get_input_port(0).get_size(),
                                        indices_, ticks_per_revolution_);
}

template class RotaryEncoders<double>;
template class RotaryEncoders<AutoDiffXd>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake

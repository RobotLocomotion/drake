#pragma once

#include "drake/lcm/translator.h"

#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {
namespace lcm {

class MyLcmtDrakeSignalTranslator : public drake::lcm::TranslatorBase<VectorBase<double>, lcmt_drake_signal> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MyLcmtDrakeSignalTranslator)
  explicit MyLcmtDrakeSignalTranslator(int size)
      : size_(size) {}

  std::unique_ptr<VectorBase<double>> AllocateDecodedData() const override;
  void InitializeMessage(lcmt_drake_signal* msg) const override;

  void Decode(const lcmt_drake_signal& msg,
      double* time, VectorBase<double>* vector_base) const override;

  void Encode(double time,
      const VectorBase<double>& vector_base,
      lcmt_drake_signal* msg) const override;

  int get_vector_size() const { return size_; }

 private:
  int size_{0};
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

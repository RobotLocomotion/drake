#pragma once

#include "drake/lcm/translator.h"

#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {
namespace lcm {

class MyLcmtDrakeSignalTranslator
    : public drake::lcm::TranslatorBase<VectorBase<double>, lcmt_drake_signal> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MyLcmtDrakeSignalTranslator)
  explicit MyLcmtDrakeSignalTranslator(int vector_size)
      : vector_size_(vector_size) {

  }

  //void InitializeMessage(lcmt_drake_signal* msg) const override;

  const TypeA& get_default_type_a() const override {

  }

  const TypeB& get_default_type_b() const override {

  }

  void AToB(const lcmt_drake_signal& msg, double* time,
            VectorBase<double>* vector_base) const override;

  void BToA(double time, const VectorBase<double>& vector_base,
            lcmt_drake_signal* msg) const override;

  int get_vector_size() const { return vector_size_; }

 private:
  int vector_size_{0};
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

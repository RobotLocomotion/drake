#include "drake/systems/lcm/lcm_and_vector_base_translator.h"

namespace drake {
namespace systems {
namespace lcm {

LcmAndVectorBaseTranslator::LcmAndVectorBaseTranslator(int size)
    : size_(size) {}

LcmAndVectorBaseTranslator::~LcmAndVectorBaseTranslator() {}

int LcmAndVectorBaseTranslator::get_vector_size() const {
  return size_;
}

std::unique_ptr<BasicVector<double>>
LcmAndVectorBaseTranslator::AllocateOutputVector() const {
  return nullptr;
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake

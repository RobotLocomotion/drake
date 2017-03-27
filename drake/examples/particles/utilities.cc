#include "drake/examples/particles/utilities.h"

namespace drake {
namespace examples {
namespace particles {

template<typename T>
std::unique_ptr<typename systems::MatrixGain<T>>
MakeDegenerateEulerJoint(const MatrixX<T>& translator) {
  // Get translation matrix dimensions.
  int row_count = translator.rows();
  int col_count = translator.cols();
  // Output degrees of freedom must be 6.
  DRAKE_DEMAND(row_count == 6);
  // Cannot have less than 1 input degree of freedom!
  DRAKE_DEMAND(col_count > 0);
  // Cannot have more than 5 input degrees of freedom!
  DRAKE_DEMAND(col_count < 6);
  // Build 12x2N matrix.
  MatrixX<T> full_translator(2 * row_count, 2 * col_count);
  full_translator.setZero();
  full_translator.topLeftCorner(row_count, col_count) = translator;
  full_translator.bottomRightCorner(row_count, col_count) = translator;
  // Return matrix gain representing the joint.
  return std::make_unique<typename systems::MatrixGain<T>>(full_translator);
}

template std::unique_ptr<systems::MatrixGain<double>>
MakeDegenerateEulerJoint<double>(const MatrixX<double>& translator);

}  // namespace particles
}  // namespace examples
}  // namespace drake

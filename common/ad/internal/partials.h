#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace ad {
namespace internal {

/* A vector of partial derivatives, for use with Drake's AutoDiff.

Partials are dynamically sized, and can have size() == 0.

When adding two Partials, they must have a compatible size(). A Partials with
size() == 0 may be freely combined with any other size, and is treated as-if
it were filled with all zeros. When adding two Partials where each one has a
non-zero size, the two sizes must be identical.

Note that a Partials object with non-zero size might still be filled with zeros
for its values. Once a non-zero size() has been established, it is "sticky" and
never returns back to being zero-sized (unless the Partials is moved-from).

In particular, note that the result of a binary operation takes on the size from
either operand, e.g., foo.Add(bar) with foo.size() == 0 and bar.size() == 4 will
will result in foo.size() == 4 after the addition, and that's true even if bar's
vector was all zeros. */
class Partials {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Partials);

  /* Constructs an empty vector. */
  Partials() = default;

  /* Constructs a single partial derivative of `coeff` at the given `offset` in
  a vector of `size` otherwise-zero derivatives.
  @throws std::exception if offset >= size */
  Partials(Eigen::Index size, Eigen::Index offset, double coeff = 1.0);

  /* Constructs a vector with a copy of the given value. */
  explicit Partials(const Eigen::Ref<const Eigen::VectorXd>& value);

  ~Partials() = default;

  /* Returns the size of this vector. */
  int size() const {
    return derivatives_.size();
  }

  /* Set this to zero. */
  void SetZero() {
    derivatives_.setZero();
  }

  /* Scales this vector by the given amount. */
  void Mul(double factor) {
    derivatives_ *= factor;
  }

  /* Scales this vector by the reciprocal of the given amount. */
  void Div(double factor) {
    derivatives_ /= factor;
  }

  /* Adds `other` into `this`. */
  void Add(const Partials& other);

  /* Adds `scale * other` into `this`. */
  void AddScaled(double scale, const Partials& other);

  /* Returns the underlying storage vector (readonly).
  TODO(jwnimmer-tri) Use a more Xpr-like return type. By "Xpr", we mean what
  Eigen calls an XprType, e.g., something like Eigen::CwiseBinaryOp. */
  const Eigen::VectorXd& make_const_xpr() const {
    return derivatives_;
  }

  /* Returns the underlying storage vector (mutable). */
  Eigen::VectorXd& get_raw_storage_mutable() {
    return derivatives_;
  }

 private:
  // TODO(jwnimmer-tri) Replace this implementation with a more efficient
  // representation.
  Eigen::VectorXd derivatives_;
};

}  // namespace internal
}  // namespace ad
}  // namespace drake

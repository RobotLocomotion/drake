#pragma once

namespace drake {
namespace math {

// We must forward-declare these classes to avoid a circular dependency.
#ifndef DRAKE_DOXYGEN_CXX
template <typename>
class RotationMatrix;
template <typename>
class RigidTransform;
#endif

namespace internal {

/* Declarations for fast, low-level functions for handling objects stored in
small matrices with known memory layouts. Ideally these are implemented using
platform-specific SIMD instructions for speed; however, we always provide a
straightforward portable fallback. */

/* Composes two RotationMatrix<double> objects as quickly as
possible, resulting in a new RotationMatrix.

Here we calculate `R_AC = R_AB * R_BC`. It is OK for R_AC to overlap
with one or both inputs. */
void ComposeRR(const RotationMatrix<double>& R_AB,
               const RotationMatrix<double>& R_BC,
               RotationMatrix<double>* R_AC);

/* Composes the inverse of a RotationMatrix<double> object with another
(non-inverted) RotationMatrix<double> as quickly as possible, resulting in a new
RotationMatrix.

@note A valid RotationMatrix is orthonormal, and the inverse of an orthonormal
matrix is just its transpose. This function assumes orthonormality and hence
simply multiplies the transpose of its first argument by the second.

Here we calculate `R_AC = R_BA⁻¹ * R_BC`. It is OK for R_AC to overlap with one
or both inputs. */
void ComposeRinvR(const RotationMatrix<double>& R_BA,
                  const RotationMatrix<double>& R_BC,
                  RotationMatrix<double>* R_AC);

/* Composes two RigidTransform<double> objects as quickly as possible, resulting
in a new RigidTransform.

@note This function is specialized for RigidTransforms and is not just a matrix
multiply.

Here we calculate `X_AC = X_AB * X_BC`. It is OK for X_AC to overlap with one or
both inputs. */
void ComposeXX(const RigidTransform<double>& X_AB,
               const RigidTransform<double>& X_BC,
               RigidTransform<double>* X_AC);

/* Composes the inverse of a RigidTransform<double> object with another
(non-inverted) RigidTransform<double> as quickly as possible, resulting in a new
RigidTransform.

@note This function is specialized for RigidTransforms and is not just a matrix
multiply.

Here we calculate `X_AC = X_BA⁻¹ * X_BC`. It is OK for X_AC to overlap with one
or both inputs. */
void ComposeXinvX(const RigidTransform<double>& X_BA,
                  const RigidTransform<double>& X_BC,
                  RigidTransform<double>* X_AC);

}  // namespace internal
}  // namespace math
}  // namespace drake

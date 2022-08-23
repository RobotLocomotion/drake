#include "drake/math/fast_pose_composition_functions.h"

#include <algorithm>
#include <cassert>
#include <type_traits>

#include "drake/math/fast_pose_composition_functions_avx2_fma.h"

/* Note that we do not include code from drake/common here so that we don't
have to fight with Eigen regarding the enabling of AVX instructions. */

namespace drake {
namespace math {
namespace internal {

/* The portable versions are always defined. They should be written
to maximize the chance that a dumb compiler can generate fast code.

The AVX functions are optionally enabled depending on the definitions used for
this compilation unit. If enabled, the no-suffix, publicly-visible functions
like ComposeRR() are implemented with the "...Avx" methods, otherwise with the
"...Portable" methods.

The AVX functions are strictly local to this file, but the portable ones are
placed in namespace internal so that they can be unit tested regardless of
whether they are used on this platform to implement the publicly-visible
functions.

Note that, except when explicitly marked "...NoAlias", the methods below must
allow for the output argument's memory to overlap with any of the input
arguments' memory.

We make judicious use below of potentially-dangerous reinterpret_casts to
convert from user-friendly APIs written in terms of RotationMatrix& and
RigidTransform& (whose declarations are necessarily unknown here) to
implementation-friendly double* types. Why this is safe here:
  - our implementations of these classes guarantee a particular memory
    layout on which we can depend,
  - the address of a class is the address of its first member (mandated by
    the standard), and
  - reinterpret_cast of a pointer to another pointer type and back yields the
    same pointer, i.e. the bit pattern does not change.
*/

namespace {
/* Dot product of a row of l and a column of m, where both l and m are 3x3s
in column order. */
double row_x_col(const double* l, const double* m) {
  return l[0] * m[0] + l[3] * m[1] + l[6] * m[2];
}

/* Dot product of a column of l and a column of m, where both l and m are 3x3s
in column order. */
double col_x_col(const double* l, const double* m) {
  return l[0] * m[0] + l[1] * m[1] + l[2] * m[2];
}

/* @pre R_AC is disjoint in memory from the inputs. */
void ComposeRRNoAlias(const double* R_AB, const double* R_BC, double* R_AC) {
  R_AC[0] = row_x_col(&R_AB[0], &R_BC[0]);
  R_AC[1] = row_x_col(&R_AB[1], &R_BC[0]);
  R_AC[2] = row_x_col(&R_AB[2], &R_BC[0]);
  R_AC[3] = row_x_col(&R_AB[0], &R_BC[3]);
  R_AC[4] = row_x_col(&R_AB[1], &R_BC[3]);
  R_AC[5] = row_x_col(&R_AB[2], &R_BC[3]);
  R_AC[6] = row_x_col(&R_AB[0], &R_BC[6]);
  R_AC[7] = row_x_col(&R_AB[1], &R_BC[6]);
  R_AC[8] = row_x_col(&R_AB[2], &R_BC[6]);
}

/* @pre R_AC is disjoint in memory from the inputs. */
void ComposeRinvRNoAlias(const double* R_BA, const double* R_BC, double* R_AC) {
  R_AC[0] = col_x_col(&R_BA[0], &R_BC[0]);
  R_AC[1] = col_x_col(&R_BA[3], &R_BC[0]);
  R_AC[2] = col_x_col(&R_BA[6], &R_BC[0]);
  R_AC[3] = col_x_col(&R_BA[0], &R_BC[3]);
  R_AC[4] = col_x_col(&R_BA[3], &R_BC[3]);
  R_AC[5] = col_x_col(&R_BA[6], &R_BC[3]);
  R_AC[6] = col_x_col(&R_BA[0], &R_BC[6]);
  R_AC[7] = col_x_col(&R_BA[3], &R_BC[6]);
  R_AC[8] = col_x_col(&R_BA[6], &R_BC[6]);
}

/* @pre X_AC is disjoint in memory from the inputs. */
void ComposeXXNoAlias(const double* X_AB, const double* X_BC, double* X_AC) {
  const double* p_AB = X_AB + 9;  // Make some nice aliases.
  const double* p_BC = X_BC + 9;
  double* p_AC = X_AC + 9;

  // X_AB * X_BC = [ R_AB; p_AB ] * [ R_BC; p_BC ]
  //             = [ (R_AB*R_BC); (p_AB + R_AB*p_BC) ]

  ComposeRRNoAlias(X_AB, X_BC, X_AC);  // Just works with first 9 elements.
  p_AC[0] = p_AB[0] + row_x_col(&X_AB[0], p_BC);
  p_AC[1] = p_AB[1] + row_x_col(&X_AB[1], p_BC);
  p_AC[2] = p_AB[2] + row_x_col(&X_AB[2], p_BC);
}

/* @pre X_AC is disjoint in memory from the inputs. */
void ComposeXinvXNoAlias(const double* X_BA, const double* X_BC,
                          double* X_AC) {
  const double* p_BA = X_BA + 9;  // Make some nice aliases.
  const double* p_BC = X_BC + 9;
  double* p_AC = X_AC + 9;

  // X_BA⁻¹ * X_BC = [ R_BA⁻¹; (R_BA⁻¹ * -p_BA) ]  * [ R_BC; p_BC ]
  //               = [ (R_BA⁻¹ * R_BC); (R_BA⁻¹ * (p_BC - p_BA)) ]

  ComposeRinvRNoAlias(X_BA, X_BC, X_AC);  // Just works with first 9 elements.
  const double p_AC_B[3] = {p_BC[0] - p_BA[0],
                            p_BC[1] - p_BA[1],
                            p_BC[2] - p_BA[2]};
  p_AC[0] = col_x_col(&X_BA[0], p_AC_B);  // Note that R_BA⁻¹ = R_BAᵀ so we
  p_AC[1] = col_x_col(&X_BA[3], p_AC_B);  // just need to use columns here
  p_AC[2] = col_x_col(&X_BA[6], p_AC_B);  // rather than rows.
}

/* Reinterpret user-friendly class names to raw arrays of double. See note above
as to why these reinterpret_casts are safe. */

const double* GetRawMatrixStart(const RotationMatrix<double>& R) {
  return reinterpret_cast<const double*>(&R);
}

double* GetMutableRawMatrixStart(RotationMatrix<double>* R) {
  return reinterpret_cast<double*>(R);
}

const double* GetRawMatrixStart(const RigidTransform<double>& X) {
  return reinterpret_cast<const double*>(&X);
}

double* GetMutableRawMatrixStart(RigidTransform<double>* X) {
  return reinterpret_cast<double*>(X);
}

/* Wrapper class to select the appropriate implementation of the composition
functions given: (1) the options enabled at build time, and (2) which features
are supported by the hardware the process is currently running on. Note that
since PoseCompositionFunctionsHelper is used as a static, it must be trivially
destructible. */
class PoseCompositionFunctionsHelper {
 public:
  PoseCompositionFunctionsHelper() {
    static_assert(
        std::is_trivially_destructible_v<PoseCompositionFunctionsHelper>,
        "PoseCompositionFunctionsHelper must be trivially destructible");
    if (internal::AvxSupported()) {
      compose_rr_ = internal::ComposeRRAvx;
      compose_rinvr_ = internal::ComposeRinvRAvx;
      compose_xx_ = internal::ComposeXXAvx;
      compose_xinvx_ = internal::ComposeXinvXAvx;
      is_using_portable_functions_ = false;
    } else {
      compose_rr_ = internal::ComposeRRPortable;
      compose_rinvr_ = internal::ComposeRinvRPortable;
      compose_xx_ = internal::ComposeXXPortable;
      compose_xinvx_ = internal::ComposeXinvXPortable;
      is_using_portable_functions_ = true;
    }
  }

  void ComposeRR(const RotationMatrix<double>& R_AB,
                 const RotationMatrix<double>& R_BC,
                 RotationMatrix<double>* R_AC) const {
    (*compose_rr_)(R_AB, R_BC, R_AC);
  }

  void ComposeRinvR(const RotationMatrix<double>& R_BA,
                    const RotationMatrix<double>& R_BC,
                    RotationMatrix<double>* R_AC) const {
    (*compose_rinvr_)(R_BA, R_BC, R_AC);
  }

  void ComposeXX(const RigidTransform<double>& X_AB,
                 const RigidTransform<double>& X_BC,
                 RigidTransform<double>* X_AC) const {
    (*compose_xx_)(X_AB, X_BC, X_AC);
  }

  void ComposeXinvX(const RigidTransform<double>& X_BA,
                    const RigidTransform<double>& X_BC,
                    RigidTransform<double>* X_AC) const {
    (*compose_xinvx_)(X_BA, X_BC, X_AC);
  }

  bool is_using_portable_functions() const {
    return is_using_portable_functions_;
  }

 private:
  void (*compose_rr_)(
      const RotationMatrix<double>&,
      const RotationMatrix<double>&,
      RotationMatrix<double>*) = nullptr;

  void (*compose_rinvr_)(
      const RotationMatrix<double>&,
      const RotationMatrix<double>&,
      RotationMatrix<double>*) = nullptr;

  void (*compose_xx_)(
      const RigidTransform<double>&,
      const RigidTransform<double>&,
      RigidTransform<double>*) = nullptr;

  void (*compose_xinvx_)(
      const RigidTransform<double>&,
      const RigidTransform<double>&,
      RigidTransform<double>*) = nullptr;

  bool is_using_portable_functions_ = false;
};

static const PoseCompositionFunctionsHelper g_pose_composition_functions_helper;

}  // namespace

/* Composition of rotation matrices R_AC = R_AB * R_BC. Each matrix is 9
consecutive doubles in column order. */
void ComposeRRPortable(const RotationMatrix<double>& R_AB,
                       const RotationMatrix<double>& R_BC,
                       RotationMatrix<double>* R_AC) {
  assert(R_AC != nullptr);
  double R_AC_temp[9];  // Protect from overlap with inputs.
  ComposeRRNoAlias(GetRawMatrixStart(R_AB), GetRawMatrixStart(R_BC), R_AC_temp);
  std::copy(R_AC_temp, R_AC_temp + 9, GetMutableRawMatrixStart(R_AC));
}

/* Composition of rotation matrices R_AC = R_BA⁻¹ * R_BC. Each matrix is 9
consecutive doubles in column order (the inverse can be viewed as the same
matrix in row order). */
void ComposeRinvRPortable(const RotationMatrix<double>& R_BA,
                          const RotationMatrix<double>& R_BC,
                          RotationMatrix<double>* R_AC) {
  assert(R_AC != nullptr);
  double R_AC_temp[9];  // Protect from overlap with inputs.
  ComposeRinvRNoAlias(GetRawMatrixStart(R_BA), GetRawMatrixStart(R_BC),
                      R_AC_temp);
  std::copy(R_AC_temp, R_AC_temp + 9, GetMutableRawMatrixStart(R_AC));
}

/* Composition of transforms X_AC = X_AB * X_BC. Rotation matrix and position
vector are adjacent in memory in 12 consecutive doubles, in column order. */
void ComposeXXPortable(const RigidTransform<double>& X_AB,
                       const RigidTransform<double>& X_BC,
                       RigidTransform<double>* X_AC) {
  assert(X_AC != nullptr);
  double X_AC_temp[12];  // Protect from overlap with inputs.
  ComposeXXNoAlias(GetRawMatrixStart(X_AB), GetRawMatrixStart(X_BC), X_AC_temp);
  std::copy(X_AC_temp, X_AC_temp + 12, GetMutableRawMatrixStart(X_AC));
}

/* Composition of transforms X_AC = X_BA⁻¹ * X_BC. Rotation matrix and position
vector are adjacent in memory in 12 consecutive doubles, in column order. */
void ComposeXinvXPortable(const RigidTransform<double>& X_BA,
                          const RigidTransform<double>& X_BC,
                          RigidTransform<double>* X_AC) {
  assert(X_AC != nullptr);
  double X_AC_temp[12];  // Protect from overlap with inputs.
  ComposeXinvXNoAlias(GetRawMatrixStart(X_BA), GetRawMatrixStart(X_BC),
                      X_AC_temp);
  std::copy(X_AC_temp, X_AC_temp + 12, GetMutableRawMatrixStart(X_AC));
}

bool IsUsingPortableCompositionFunctions() {
  return g_pose_composition_functions_helper.is_using_portable_functions();
}

void ComposeRR(const RotationMatrix<double>& R_AB,
               const RotationMatrix<double>& R_BC,
               RotationMatrix<double>* R_AC) {
  g_pose_composition_functions_helper.ComposeRR(R_AB, R_BC, R_AC);
}

void ComposeRinvR(const RotationMatrix<double>& R_BA,
                  const RotationMatrix<double>& R_BC,
                  RotationMatrix<double>* R_AC) {
  g_pose_composition_functions_helper.ComposeRinvR(R_BA, R_BC, R_AC);
}

void ComposeXX(const RigidTransform<double>& X_AB,
               const RigidTransform<double>& X_BC,
               RigidTransform<double>* X_AC) {
  g_pose_composition_functions_helper.ComposeXX(X_AB, X_BC, X_AC);
}

void ComposeXinvX(const RigidTransform<double>& X_BA,
                  const RigidTransform<double>& X_BC,
                  RigidTransform<double>* X_AC) {
  g_pose_composition_functions_helper.ComposeXinvX(X_BA, X_BC, X_AC);
}

}  // namespace internal
}  // namespace math
}  // namespace drake

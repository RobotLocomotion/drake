#include "drake/geometry/proximity/simd_avx.h"

#include <immintrin.h>

// Warning: This is an experimental code to use AVX instructions. One
// limitation is that we cannot use Eigen in this file due to incompatible ABI.
// That's the API uses double* instead of Vector3d or Matrix3d.

namespace drake {
namespace geometry {
namespace internal {

// Dot product of a row of l and a column of r, where both l and r are 3x3s in
// column order.
static double row_x_col_NoAVX(const double* l, const double* m) {
  return l[0] * m[0] + l[3] * m[1] + l[6] * m[2];
}

// Straightforward 3x3 multiply: a = l*r. Matrices in column order.
static void mult3x3_NoAVX(const double* l, const double* m, double* o) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      const int c = 3 * j;  // Column start index.
      o[c + i] = row_x_col_NoAVX(&l[i], &m[c]);
    }
}

// Compose RigidTransforms stored as 3x3 RotationMatrix and separate position
// vector. On return R_AC=R_AB*R_BC and p_AC = p_AB + p_BC_A where
// p_BC_A=R_AB*p_BC.
// I cannot declare `static` for this function. Otherwise, I got "error
// unused function."
void multX_NoAVX(const double* R_AB, const double* p_AB, const double* R_BC,
                 const double* p_BC, double* R_AC, double* p_AC) {
  mult3x3_NoAVX(R_AB, R_BC, R_AC);
  for (int i = 0; i < 3; ++i)
    p_AC[i] = p_AB[i] + row_x_col_NoAVX(&R_AB[i], p_BC);
}

void multX2(const double* a /*R_AB*/, const double* x /*p_AB*/,
            const double* A /*R_BC*/, const double* X /*p_BC*/,
            double* r /*R_AC*/, double* xx /*p_AC*/) {
#ifdef __APPLE__
  multX_NoAVX(a, x, A, X, r, xx);
}
#else
  // constexpr uint64_t yes = 0x8000000000000000ULL;
  constexpr u_int64_t yes  = u_int64_t(1ull << 63);
  constexpr u_int64_t no  = u_int64_t(0);
  const __m256i mask = _mm256_setr_epi64x(yes, yes, yes, no);
  const __m256d col0 = _mm256_loadu_pd(a);    // a b c (d)  d is unused
  const __m256d col1 = _mm256_loadu_pd(a+3);  // d e f (g)  g is unused
  const __m256d col2 = _mm256_maskload_pd(a+6, mask);  // g h i (0)
  // Column rst
  __m256d dup0 = _mm256_broadcast_sd(A);    // A A A A
  __m256d dup1 = _mm256_broadcast_sd(A+1);  // B B B B
  __m256d dup2 = _mm256_broadcast_sd(A+2);  // C C C C
  __m256d res = _mm256_mul_pd(col0, dup0);  // aA bA cA (dA)
  // aA+dB bA+eB cA+fB (dA+gB)
  res = _mm256_fmadd_pd(col1, dup1, res);
  // aA+dB+gC bA+eB+hC cA+fB+iC (dA+gB+0C)
  res = _mm256_fmadd_pd(col2, dup2, res);
  _mm256_storeu_pd(r, res);                // r s t (u)  will overwrite u
  // Column uvw
  dup0 = _mm256_broadcast_sd(A+3);         // D D D D
  dup1 = _mm256_broadcast_sd(A+4);         // E E E E
  dup2 = _mm256_broadcast_sd(A+5);         // F F F F
  res = _mm256_mul_pd(col0, dup0);         // aD bD cD (dD)
  res = _mm256_fmadd_pd(col1, dup1, res);  // aD+dE bD+eE cD+fE (dD+gE)
  // aD+dE+gF bD+eE+hF cD+fE+iF (dD+gE+0F)
  res = _mm256_fmadd_pd(col2, dup2, res);
  _mm256_storeu_pd(r+3, res);              // u v w (x)  will overwrite
  // Column xyz
  dup0 = _mm256_broadcast_sd(A+6);         // G G G G
  dup1 = _mm256_broadcast_sd(A+7);         // H H H H
  dup2 = _mm256_broadcast_sd(A+8);         // I I I I
  res = _mm256_mul_pd(col0, dup0);         // aG bG cG (dG)
  res = _mm256_fmadd_pd(col1, dup1, res);  // aG+dH bG+eH cG+fH (dG+gH)
  // aG+dH+gI bG+eH+hI cG+fH+iI (dG+gH+0I)
  res = _mm256_fmadd_pd(col2, dup2, res);
  _mm256_maskstore_pd(r+6, mask, res);      // x y z
  // Column xx yy zz
  dup0 = _mm256_broadcast_sd(X);            // X X X X
  dup1 = _mm256_broadcast_sd(X+1);          // Y Y Y Y
  dup2 = _mm256_broadcast_sd(X+2);          // Z Z Z Z
  const __m256d col3 = _mm256_maskload_pd(x, mask);  // x y z (0)
  res = _mm256_fmadd_pd(col0, dup0, col3);  // x+aX y+bX z+cX (0+dX)
  // x+aX+dY    y+bX+eY    z+cX+fY    (0+dX+gY)
  res = _mm256_fmadd_pd(col1, dup1, res);
  // x+aX+dY+gZ y+bX+eY+hZ z+cX+fY+iZ (0+dX+gY+0Z)
  res = _mm256_fmadd_pd(col2, dup2, res);
  _mm256_maskstore_pd(xx, mask, res);       // xx yy zz
}
#endif

}  // namespace internal
}  // namespace geometry
}  // namespace drake


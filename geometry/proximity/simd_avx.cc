#include "drake/geometry/proximity/simd_avx.h"

#include <cstdint>

#include <immintrin.h>

// Warning: This is an experimental code to use AVX instructions. One
// limitation is that we cannot use Eigen in this file due to incompatible ABI.
// That's the API uses double* instead of Vector3d or Matrix3d.

namespace drake {
namespace geometry {
namespace internal {

#if !(defined(__FMA__) && defined(__AVX2__))

// Dot product of a row of l and a column of r, where both l and r are 3x3s in
// column order.
static double row_x_col_NoAVX(const double* l, const double* m) {
  return l[0] * m[0] + l[3] * m[1] + l[6] * m[2];
}

// Straightforward 3x3 multiply: a = l*r. Matrices in column order.
static inline void mult3x3_NoAVX(const double* l, const double* m, double* o) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      const int c = 3 * j;  // Column start index.
      o[c + i] = row_x_col_NoAVX(&l[i], &m[c]);
    }
}

// Compose RigidTransforms stored as 3x3 RotationMatrix and separate position
// vector. On return R_AC=R_AB*R_BC and p_AC = p_AB + p_BC_A where
// p_BC_A=R_AB*p_BC.
static inline void multX2_NoAVX(const double* R_AB, const double* p_AB,
                                const double* R_BC, const double* p_BC,
                                double* R_AC, double* p_AC) {
  mult3x3_NoAVX(R_AB, R_BC, R_AC);
  for (int i = 0; i < 3; ++i)
    p_AC[i] = p_AB[i] + row_x_col_NoAVX(&R_AB[i], p_BC);
}

void multX2(const double* a /*R_AB*/, const double* x /*p_AB*/,
            const double* A /*R_BC*/, const double* X /*p_BC*/,
            double* r /*R_AC*/, double* xx /*p_AC*/) {
  multX2_NoAVX(a, x, A, X, r, xx);
}

void multXX(const double* a /*X_AB*/, const double* A /*X_BC*/,
            double* r /*X_AC*/) {
  multX2_NoAVX(a, a + 9, A, A + 9, r, r + 9);
}

// Straightforward matrix-vector multiplication between 3x3-matrix M and
// 3-vector V. The result is in 3-vector r.  The 3x3 matrix M is stored as
// the array M[9] in column order. For example, the first column M(*,0) is:
// M(0,0) = M[0],
// M(1,0) = M[1],
// M(2,0) = M[2],
static inline void multMV_NoAVX(const double* M, const double* V,
                                double* r) {
  r[0] = row_x_col_NoAVX(M, V);
  r[1] = row_x_col_NoAVX(M + 1, V);
  r[2] = row_x_col_NoAVX(M + 2, V);
}

static inline void multXinvX_NoAVX(const double* X_BA, const double* X_BC,
                                   double* r /*X_AC*/) {
  // Calculate inverse rigid transform X_AB of X_BA.
  double X_AB[12];
  {
    // Alias rotation R as the first 9 entries of rigid transform X.
    const double* R_BA = X_BA;
    double* R_AB = X_AB;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R_AB[i + 3 * j] = R_BA[j + 3 * i];
      }
    }
    // Alias the translation vector as the last 3 entries of rigid transform.
    const double* p_BoAo_B = X_BA + 9;
    double* p_AB = X_AB + 9;
    const double p_AoBo_B[3] = {-p_BoAo_B[0], -p_BoAo_B[1], -p_BoAo_B[2]};
    multMV_NoAVX(R_AB, p_AoBo_B, p_AB);
  }

  multXX(X_AB, X_BC, r);
}

void multXinvX(const double* X_BA, const double* X_BC, double* r /*X_AC*/) {
  multXinvX_NoAVX(X_BA, X_BC, r /*X_AC*/);
}

#else    // Use AVX instructions

void multX2(const double* a /*R_AB*/, const double* x /*p_AB*/,
            const double* A /*R_BC*/, const double* X /*p_BC*/,
            double* r /*R_AC*/, double* xx /*p_AC*/) {
  // constexpr uint64_t yes = 0x8000000000000000ULL;
  constexpr uint64_t yes = uint64_t(1ull << 63);
  constexpr uint64_t no  = uint64_t(0);
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

// Turn d into d d d d.
static inline __m256d four(double d) {
  return _mm256_set1_pd(d);
}

// Turn *d into *d *d *d *d.
static inline __m256d four(const double* d) {
  return _mm256_broadcast_sd(d);
}

/* Composition of transforms X_AC=X_AB*X_BC.
Rotation matrix and position vector are adjacent in memory.

We want to perform this 3x3 matrix multiply:

     R_AC    R_AB    R_BC

    r u x   a d g   A D G
    s v y = b e h * B E H     All column ordered in memory.
    t w z   c f i   C F I

and    xx   x   a d g   X
       yy = y + b e h * Y
       zz   z   c f i   Z

This is 63 flops altogether.

Strategy: compute column rst in parallel, then uvw, then xyz.
r = aA+dB+gC          xx = x + aX + dY + gZ
s = bA+eB+hC  etc.    yy = y + bX + eY + hZ
t = cA+fB+iC          zz = z + cX + fY + iZ

Load columns from left matrix, duplicate elements from right.
Peform 4 operations in parallel but ignore the 4th result.
Be careful not to load or store past the last element.

We end up doing 3*4 + 9*8 = 84 flops to get 63 useful ones.
However, we do that in only 12 floating point instructions
(three packed multiplies, nine packed fused-multiply-adds).
*/

// Try multiple accumulators.
void multXX(const double* a /*X_AB*/, const double* A /*X_BC*/,
            double* r /*X_AC*/) {
  constexpr uint64_t yes  = uint64_t(1ull << 63);
  constexpr uint64_t no  = uint64_t(0);
  const __m256i mask = _mm256_setr_epi64x(yes, yes, yes, no);
  __m256d acc0, acc1, acc2, acc3;  // Accumulators.

  const __m256d abcd = _mm256_loadu_pd(a);            // a b c (d)  d is unused
  const __m256d xyz0 = _mm256_maskload_pd(a+9, mask);  // x y z (0)

  acc0 = _mm256_mul_pd(abcd, four(A));            // aA bA cA (dA)
  acc1 = _mm256_mul_pd(abcd, four(A+3));          // aD bD cD (dD)
  acc2 = _mm256_mul_pd(abcd, four(A+6));          // aG bG cG (dG)
  acc3 = _mm256_fmadd_pd(abcd, four(A+9), xyz0);  // x+aX y+bX z+cX (0+dX)

  const __m256d defg = _mm256_loadu_pd(a+3);      // d e f (g)  g is unused
  acc0 = _mm256_fmadd_pd(defg, four(A+1), acc0);  // aA+dB bA+eB cA+fB (dA+gB)
  acc1 = _mm256_fmadd_pd(defg, four(A+4), acc1);  // aD+dE bD+eE cD+fE (dD+gE)
  acc2 = _mm256_fmadd_pd(defg, four(A+7), acc2);  // aG+dH bG+eH cG+fH (dG+gH)
  // x+aX+dY    y+bX+eY    z+cX+fY    (0+dX+gY)
  acc3 = _mm256_fmadd_pd(defg, four(A+10), acc3);

  const __m256d ghix = _mm256_loadu_pd(a+6);      // g h i (x)
  // aA+dB+gC bA+eB+hC cA+fB+iC (dA+gB+xC)
  acc0 = _mm256_fmadd_pd(ghix, four(A+2), acc0);
  // r s t (u)  we will overwrite u
  _mm256_storeu_pd(r, acc0);

  // aD+dE+gF bD+eE+hF cD+fE+iF (dD+gE+0F)
  acc1 = _mm256_fmadd_pd(ghix, four(A+5), acc1);
  _mm256_storeu_pd(r+3, acc1);  // u v w (x)  we will overwrite x

  // aG+dH+gI bG+eH+hI cG+fH+iI (dG+gH+0I)
  acc2 = _mm256_fmadd_pd(ghix, four(A+8), acc2);
  _mm256_storeu_pd(r+6, acc2);  // x y z (xx)  we will overwrite xx

  // x+aX+dY+gZ y+bX+eY+hZ z+cX+fY+iZ (0+dX+gY+xZ)
  acc3 = _mm256_fmadd_pd(ghix, four(A+11), acc3);
  _mm256_maskstore_pd(r+9, mask, acc3);  // xx yy zz

  _mm256_zeroupper();
}

/* Composition of transforms X_AC=X_BA⁻¹*X_BC.
Rotation matrix and position vector are adjacent in memory.

We want to perform this 3x3 matrix multiply:

     R_AC    R_BA⁻¹  R_BC

    r u x   a b c   A D G
    s v y = d e f * B E H     R_BA⁻¹ is row ordered; R_BC col ordered
    t w z   g h i   C F I

and    xx   a b c     X   x
       yy = d e f * ( Y − y )
       zz   g h i     Z   z

This is 63 flops altogether.

Strategy: compute column rst in parallel, then uvw, then xyz.
Let PQR=XYZ-xyz.
r = aA+bB+cC          xx = aP + bQ + cR
s = dA+eB+fC  etc.    yy = dP + eQ + fR
t = gA+hB+iC          zz = gP + hQ + iR

Load columns from left matrix, duplicate elements from right.
Peform 4 operations in parallel but ignore the 4th result.
Be careful not to load or store past the last element.

We end up doing 5*4 + 8*8 = 84 flops to get 63 useful ones.
However, we do that in only 13 floating point instructions
(4 packed multiplies, 1 packed subtract, 8 packed fused-multiply-adds).

This method is about 2X slower than multXX when compiled with
g++ 7.5, but is almost full speed with clang++ 6.
*/
void multXinvX(const double* a /*X_BA*/, const double* A /*X_BC*/,
               double* r /*X_AC*/) {
  constexpr uint64_t yes  = uint64_t(1ull << 63);
  constexpr uint64_t no  = uint64_t(0);
  const __m256i mask = _mm256_setr_epi64x(yes, yes, yes, no);

  __m256d acc0, acc1, acc2, acc3;  // Accumulators.

  const __m256d abcd = _mm256_loadu_pd(a);
  const __m256d efgh = _mm256_loadu_pd(a+4);
  const __m256d ebgh = _mm256_blend_pd(abcd, efgh, 0b1101);  // e b g h
  const __m256d col1 = _mm256_permute_pd(ebgh, 0b0101);  // b e h (g)

  const __m256d cdgh = _mm256_permute2f128_pd(abcd, efgh, 0b00110001);
  const __m256d col0 = _mm256_blend_pd(abcd, cdgh, 0b1110);  // a d g (h) fast

  const __m256d fghi = _mm256_loadu_pd(a+5);
  const __m256d fcih = _mm256_shuffle_pd(fghi, cdgh, 0b1100);  // f c i h
  const __m256d col2 = _mm256_permute_pd(fcih, 0b1001);  // c f i (h)

  // Calculate PQR
  const __m256d loadX = _mm256_maskload_pd(A+9, mask);  // X Y Z (0)
  const __m256d loadx = _mm256_maskload_pd(a+9, mask);  // x y z (0)
  const __m256d PQR = _mm256_sub_pd(loadX, loadx);  // PQR0 = XYZ0 − xyz0 (0)

  acc0 = _mm256_mul_pd(col0, four(A));         // aA dA gA (bA)
  acc1 = _mm256_mul_pd(col0, four(A+3));       // aD dD gD (bD)
  acc2 = _mm256_mul_pd(col0, four(A+6));       // aG dG gG (bG)
  acc3 = _mm256_mul_pd(col0, four(PQR[0]));    // aP dP gP (bP)

  acc0 = _mm256_fmadd_pd(col1, four(A+1), acc0);   // aA+bB dA+eB gA+hB (bA+cB)
  acc1 = _mm256_fmadd_pd(col1, four(A+4), acc1);   // aD+bE dD+eE gD+hE (bD+cE)
  acc2 = _mm256_fmadd_pd(col1, four(A+7), acc2);   // aG+bH dG+eH gG+hH (bG+cH)
  // aP+bQ dP+eQ gP+hQ (bP+cQ)
  acc3 = _mm256_fmadd_pd(col1, four(PQR[1]), acc3);

  // aA+bB+cC dA+eB+fC gA+hB+iC (bA+cB+0C)
  acc0 = _mm256_fmadd_pd(col2, four(A+2), acc0);
  // r s t (u)  we will overwrite u
  _mm256_storeu_pd(r, acc0);

  // aD+bE+cF dD+eE+fF gD+hE+iF (bD+cE+0F)
  acc1 = _mm256_fmadd_pd(col2, four(A+5), acc1);
  _mm256_storeu_pd(r+3, acc1);  // u v w (x)  we will overwrite x

  // aG+bH+cI dG+eH+fI gG+hH+iI (bG+cH+0I)
  acc2 = _mm256_fmadd_pd(col2, four(A+8), acc2);
  _mm256_maskstore_pd(r+6, mask, acc2);  // x y z

  // aP+bQ+cR dP+eQ+fR gP+hQ+iR (bP+cQ+0R)
  acc3 = _mm256_fmadd_pd(col2, four(PQR[2]), acc3);
  _mm256_maskstore_pd(r+9, mask, acc3);  // xx yy zz

  _mm256_zeroupper();
}

#endif  // Use AVX instructions

}  // namespace internal
}  // namespace geometry
}  // namespace drake


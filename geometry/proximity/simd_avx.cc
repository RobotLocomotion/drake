#include "drake/geometry/proximity/simd_avx.h"

#include <immintrin.h>

// Warning: This is an experimental code to use AVX instructions. One
// limitation is that we cannot use Eigen in this file due to incompatible ABI.
// That's the API uses double* instead of Vector3d or Matrix3d.

namespace drake {
namespace geometry {
namespace internal {

void multX2(const double* a /*R_AB*/, const double* x /*p_AB*/,
            const double* A /*R_BC*/, const double* X /*p_BC*/,
            double* r /*R_AC*/, double* xx /*p_AC*/) {
  //constexpr uint64_t yes = 0x8000000000000000ULL;                                         //NOLINT(*)
  constexpr u_int64_t yes  = u_int64_t(1ull << 63);                                         //NOLINT(*)
  constexpr u_int64_t no  = u_int64_t(0);                                                   //NOLINT(*)
  const __m256i mask = _mm256_setr_epi64x(yes, yes, yes, no);                               //NOLINT(*)
  const __m256d col0 = _mm256_loadu_pd(a);   // a b c (d)  d is unused                      //NOLINT(*)
  const __m256d col1 = _mm256_loadu_pd(a+3); // d e f (g)  g is unused                      //NOLINT(*)
  const __m256d col2 = _mm256_maskload_pd(a+6, mask); // g h i (0)                          //NOLINT(*)
  // Column rst
  __m256d dup0 = _mm256_broadcast_sd(A);   // A A A A                                       //NOLINT(*)
  __m256d dup1 = _mm256_broadcast_sd(A+1); // B B B B                                       //NOLINT(*)
  __m256d dup2 = _mm256_broadcast_sd(A+2); // C C C C                                       //NOLINT(*)
  __m256d res = _mm256_mul_pd(col0, dup0); // aA bA cA (dA)                                 //NOLINT(*)
  res = _mm256_fmadd_pd(col1, dup1, res); // aA+dB bA+eB cA+fB (dA+gB)                      //NOLINT(*)
  res = _mm256_fmadd_pd(col2, dup2, res); // aA+dB+gC bA+eB+hC cA+fB+iC (dA+gB+0C)          //NOLINT(*)
  _mm256_storeu_pd(r, res);               // r s t (u)  will overwrite u                    //NOLINT(*)
  // Column uvw
  dup0 = _mm256_broadcast_sd(A+3);        // D D D D                                        //NOLINT(*)
  dup1 = _mm256_broadcast_sd(A+4);        // E E E E                                        //NOLINT(*)
  dup2 = _mm256_broadcast_sd(A+5);        // F F F F                                        //NOLINT(*)
  res = _mm256_mul_pd(col0, dup0);        // aD bD cD (dD)                                  //NOLINT(*)
  res = _mm256_fmadd_pd(col1, dup1, res); // aD+dE bD+eE cD+fE (dD+gE)                      //NOLINT(*)
  res = _mm256_fmadd_pd(col2, dup2, res); // aD+dE+gF bD+eE+hF cD+fE+iF (dD+gE+0F)          //NOLINT(*)
  _mm256_storeu_pd(r+3, res);             // u v w (x)  will overwrite                      //NOLINT(*)
  // Column xyz
  dup0 = _mm256_broadcast_sd(A+6);        // G G G G                                        //NOLINT(*)
  dup1 = _mm256_broadcast_sd(A+7);        // H H H H                                        //NOLINT(*)
  dup2 = _mm256_broadcast_sd(A+8);        // I I I I                                        //NOLINT(*)
  res = _mm256_mul_pd(col0, dup0);        // aG bG cG (dG)                                  //NOLINT(*)
  res = _mm256_fmadd_pd(col1, dup1, res); // aG+dH bG+eH cG+fH (dG+gH)                      //NOLINT(*)
  res = _mm256_fmadd_pd(col2, dup2, res); // aG+dH+gI bG+eH+hI cG+fH+iI (dG+gH+0I)          //NOLINT(*)
  _mm256_maskstore_pd(r+6, mask, res);    // x y z                                          //NOLINT(*)
  // Column xx yy zz
  dup0 = _mm256_broadcast_sd(X);            // X X X X                                      //NOLINT(*)
  dup1 = _mm256_broadcast_sd(X+1);          // Y Y Y Y                                      //NOLINT(*)
  dup2 = _mm256_broadcast_sd(X+2);          // Z Z Z Z                                      //NOLINT(*)
  const __m256d col3 = _mm256_maskload_pd(x, mask); // x y z (0)                            //NOLINT(*)
  res = _mm256_fmadd_pd(col0, dup0, col3);// x+aX y+bX z+cX (0+dX)                          //NOLINT(*)
  res = _mm256_fmadd_pd(col1, dup1, res); // x+aX+dY    y+bX+eY    z+cX+fY    (0+dX+gY)     //NOLINT(*)
  res = _mm256_fmadd_pd(col2, dup2, res); // x+aX+dY+gZ y+bX+eY+hZ z+cX+fY+iZ (0+dX+gY+0Z)  //NOLINT(*)
  _mm256_maskstore_pd(xx, mask, res);     // xx yy zz                                       //NOLINT(*)
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake


#pragma once

namespace drake {
namespace geometry {
namespace internal {

void multX2(const double* a /*R_AB*/, const double* x /*p_AB*/,
            const double* A /*R_BC*/, const double* X /*p_BC*/,
            double* r /*R_AC*/, double* xx /*p_AC*/);

}  // namespace internal
}  // namespace geometry
}  // namespace drake


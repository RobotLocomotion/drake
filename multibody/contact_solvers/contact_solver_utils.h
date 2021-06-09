#pragma once

#include <type_traits>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// Given a contact vector xc of size 3nc, this method copies the normal
// components of xc into xn, of size nc. Refer to MergeNormalAndTangent() for
// details on storage order and conventions.
// @pre xn is not nullptr.
// @pre xn has size nc and xc has size 3nc.
// @pre xn_t and xc_t are Eigen vector types.
template <typename xc_t, typename xn_t>
void ExtractNormal(const Eigen::MatrixBase<xc_t>& xc,
                   Eigen::MatrixBase<xn_t>* xn) {
  static_assert(is_eigen_vector<xc_t>{});
  static_assert(is_eigen_vector<xn_t>{});
  static_assert(std::is_same_v<typename xc_t::Scalar, typename xn_t::Scalar>);
  const int num_contacts = xn->size();
  DRAKE_DEMAND(xc.size() == 3 * num_contacts);
  for (int i = 0; i < num_contacts; ++i) {
    (*xn)(i) = xc(3 * i + 2);
  }
}

// Given a contact vector xc of size 3nc, this method copies the tangential
// components of xc into xt, of size 2nc. Refer to MergeNormalAndTangent() for
// details on storage order and conventions.
// @pre xt is not nullptr.
// @pre xt has size 2nc and xc has size 3nc.
// @pre xt_t and xc_t are Eigen vector types.
template <typename xc_t, typename xt_t>
void ExtractTangent(const Eigen::MatrixBase<xc_t>& xc,
                    Eigen::MatrixBase<xt_t>* xt) {
  static_assert(is_eigen_vector<xc_t>{});
  static_assert(is_eigen_vector<xt_t>{});
  static_assert(std::is_same_v<typename xc_t::Scalar, typename xt_t::Scalar>);
  DRAKE_DEMAND(xc.size() % 3 == 0);
  const int num_contacts = xc.size() / 3;
  DRAKE_DEMAND(xt->size() == 2 * num_contacts);
  for (int i = 0; i < num_contacts; ++i) {
    xt->template segment<2>(2 * i) = xc.template segment<2>(3 * i);
  }
}

// Helper to merge two vectors xn and xt storing the normal and tangential
// components respectively into a single contact vector xc.
//
// For a problem with nc contact points xc is the concatenation of nc
// vectors xcᵢ ∈ ℝ³ thus having size 3nc, `xc = (xc₁, xc₂,...,xcₙ)`. The first
// two entries in xcᵢ are expected to correspond to the tangential component
// xtᵢ ∈ ℝ² and the last and third entry is expected to correspond to the
// normal component xnᵢ ∈ ℝ. That is, xcᵢ = (xtᵢ, xnᵢ).
// xn is the concatenation xn = (xn₁, xn₂,...,xnₙ) of the scalar normal
// components xnᵢ of xcᵢ, thus xn has size nc.
// xt is the concatenation xt = (xt₁, xt₂,...,xtₙ) of the nc tangential
// components xtᵢ ∈ ℝ², thus xt has size 2nc.
//
// @pre xc is not nullptr.
// @pre xn has size nc, xt has size 2nc and xc has size 3nc.
// @pre xn_t, xt_t and xc_t are Eigen vector types.
template <typename xn_t, typename xt_t, typename xc_t>
void MergeNormalAndTangent(const Eigen::MatrixBase<xn_t>& xn,
                           const Eigen::MatrixBase<xt_t>& xt,
                           Eigen::MatrixBase<xc_t>* xc) {
  static_assert(is_eigen_vector<xc_t>{});
  static_assert(is_eigen_vector<xn_t>{});
  static_assert(is_eigen_vector<xt_t>{});
  const int num_contacts = xn.size();
  DRAKE_DEMAND(xt.size() == 2 * num_contacts);
  DRAKE_DEMAND(xc->size() == 3 * num_contacts);
  for (int i = 0; i < num_contacts; ++i) {
    xc->template segment<2>(3 * i) = xt.template segment<2>(2 * i);
    (*xc)(3 * i + 2) = xn(i);
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

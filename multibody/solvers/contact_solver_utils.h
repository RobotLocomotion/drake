#pragma once

#include <type_traits>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace solvers {

template <typename xc_t, typename xn_t>
void ExtractNormal(const Eigen::MatrixBase<xc_t>& xc,
                   Eigen::MatrixBase<xn_t>* xn) {
  static_assert(is_eigen_vector<xc_t>{});
  static_assert(is_eigen_vector<xn_t>{});
  static_assert(std::is_same<typename xc_t::Scalar, typename xn_t::Scalar>{});
  const int num_contacts = xn->size();
  DRAKE_DEMAND(xc.size() == 3 * num_contacts);
  for (int i = 0; i < num_contacts; ++i) {
    (*xn)(i) = xc(3 * i + 2);
  }
}

template <typename xc_t, typename xt_t>
void ExtractTangent(const Eigen::MatrixBase<xc_t>& xc,
                    Eigen::MatrixBase<xt_t>* xt) {
  static_assert(is_eigen_vector<xc_t>{});
  static_assert(is_eigen_vector<xt_t>{});
  static_assert(std::is_same<typename xc_t::Scalar, typename xt_t::Scalar>{});
  DRAKE_DEMAND(xc.size() % 3 == 0);
  const int num_contacts = xc.size() / 3;
  DRAKE_DEMAND(xt->size() == 2 * num_contacts);
  for (int i = 0; i < num_contacts; ++i) {
    xt->template segment<2>(2 * i) = xc.template segment<2>(3 * i);
  }
}

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

}  // namespace solvers
}  // namespace multibody
}  // namespace drake
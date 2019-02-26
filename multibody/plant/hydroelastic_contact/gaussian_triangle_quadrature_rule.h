#pragma once

#include <algorithm>
#include <vector>

#include "drake/multibody/plant/hydroelastic_contact/triangle_quadrature_rule.h"

namespace drake {
namespace multibody {
namespace hydroelastic_contact {

class GaussianTriangleQuadratureRule : public TriangleQuadratureRule {
 public:
  /// Constructs the Gaussian quadrature rule of the specified order.
  explicit GaussianTriangleQuadratureRule(int order) : order_(order) {
    if (order >= 9) {
      throw std::logic_error(
        "Gaussian triangle quadrature only supported up to fifth order "
        "presently.");
    }
    SetWeightsAndQuadraturePoints();
  }

 private:
  // Sets the weights and quadrature points depending on the order of the
  // quadrature rule. Weights and quadrature points taken from pp. 18-19 of:
  // http://math2.uncc.edu/~shaodeng/TEACHING/math5172/Lectures/Lect_15.PDF
  void SetWeightsAndQuadraturePoints() {
    switch (order_) {
      case 1:
        weights_.resize(1);
        quadrature_points_.resize(1);
        weights_[0] = 1.0;
        quadrature_points_[0] = { 1.0/3.0, 1.0/3.0 };
        break;

      case 2:
        weights_.resize(3);
        quadrature_points_.resize(3);
        weights_[0] = weights_[1] = weights_[2] = 1.0/3.0;
        quadrature_points_[0] = { 1.0/6.0, 1.0/6.0 };
        quadrature_points_[1] = { 1.0/6.0, 2.0/3.0 };
        quadrature_points_[2] = { 2.0/3.0, 1.0/6.0 };
        break;

      case 3:
        weights_.resize(4);
        quadrature_points_.resize(4);
        weights_[0] = -0.5625;
        weights_[1] = weights_[2] = weights_[3] = 0.52083333333333;
        quadrature_points_[0] = { 1.0/3.0, 1.0/3.0 };
        quadrature_points_[1] = { 0.2, 0.2 };
        quadrature_points_[2] = { 0.2, 0.6 };
        quadrature_points_[3] = { 0.6, 0.2 };
        break;

      case 4:
        weights_.resize(6);
        quadrature_points_.resize(6);
        weights_[0] = weights_[1] = weights_[2] = 0.22338158967801;
        weights_[3] = weights_[4] = weights_[5] = 0.10995174365532;
        quadrature_points_[0] = { 0.44594849091597, 0.44594849091597 };
        quadrature_points_[1] = { 0.44594849091597, 0.10810301816807 };
        quadrature_points_[2] = { 0.10810301816807, 0.44594849091597 };
        quadrature_points_[3] = { 0.09157621350977, 0.09157621350977 };
        quadrature_points_[4] = { 0.09157621350977, 0.81684757298046 };
        quadrature_points_[5] = { 0.81684757298046, 0.09157621350977 };
        break;

      case 5:
        weights_.resize(7);
        quadrature_points_.resize(7);
        weights_[0] = 0.225;
        weights_[1] = weights_[2] = weights_[3] = 0.13239415278851;
        weights_[4] = weights_[5] = weights_[6] = 0.12593918054483;
        quadrature_points_[0] = { 1.0/3.0, 1.0/3.0 };
        quadrature_points_[1] = { 0.47014206410511, 0.47014206410511 };
        quadrature_points_[2] = { 0.47014206410511, 0.05971587178977 };
        quadrature_points_[3] = { 0.05971587178977, 0.47014206410511 };
        quadrature_points_[4] = { 0.10128650732346, 0.10128650732346 };
        quadrature_points_[5] = { 0.10128650732346, 0.79742698535309 };
        quadrature_points_[6] = { 0.79742698535309, 0.10128650732346 };
        break;
    }
  }

  int do_order() const override { return order_; }

  const std::vector<double>& do_weights() const override {
    return weights_;
  }

  const std::vector<Vector2<double>>& do_quadrature_points() const override {
    return quadrature_points_;
  }

  const int order_{-1};
  std::vector<double> weights_;
  std::vector<Vector2<double>> quadrature_points_;
};

}  // namespace hydroelastic_contact
}  // namespace multibody
}  // namespace drake


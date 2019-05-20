#include "drake/geometry/query_results/mesh_field.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/geometry/query_results/surface_mesh.h"

namespace drake {
namespace geometry {
namespace {

// Tests that the base class MeshField::Clone() calls DoClone() of its
// subclass. We use `double` and SurfaceMesh<double> to represent the type
// parameters FieldValue and MeshType respectively.
GTEST_TEST(MeshFieldTest, TestClone) {
  using FieldValue = double;
  using MeshType = SurfaceMesh<double>;
  using MeshFieldBase = MeshField<FieldValue, MeshType>;

  class MeshFieldSubclass : public MeshFieldBase {
   public:
    FieldValue Evaluate(const MeshType::ElementIndex,
                        const MeshType::Barycentric&) const override {
      return FieldValue(0);
    }
   private:
    DRAKE_NODISCARD std::unique_ptr<MeshFieldBase> DoClone() const override {
      return std::unique_ptr<MeshFieldBase>(new MeshFieldSubclass());
    }
  };
  MeshFieldSubclass original;
  std::unique_ptr<MeshFieldBase> clone = original.Clone();
  EXPECT_NE(nullptr, dynamic_cast<MeshFieldSubclass*>(clone.get()));
}

}  // namespace
}  // namespace geometry
}  // namespace drake

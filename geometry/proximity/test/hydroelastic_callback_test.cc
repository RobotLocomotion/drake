#include "drake/geometry/proximity/hydroelastic_callback.h"

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/autodiff.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {
namespace {

using fcl::Boxd;
using fcl::CollisionObjectd;
using fcl::Halfspaced;
using fcl::Sphered;
using math::RigidTransform;
using math::RollPitchYaw;
using math::RotationMatrix;
using std::make_shared;
using std::make_unique;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

// Creates proximity properties sufficient to create a rigid representation
// for supported geometries.
ProximityProperties rigid_properties() {
  ProximityProperties props;
  const double resolution_hint = 0.1;
  AddRigidHydroelasticProperties(resolution_hint, &props);
  return props;
}

// Creates proximity properties sufficient to create a soft representation
// for supported geometries.
ProximityProperties soft_properties() {
  ProximityProperties props;
  const double resolution_hint = 0.25;
  AddSoftHydroelasticProperties(resolution_hint, 1e8, &props);
  // Redundantly add slab thickness so it can be used with compliant mesh or
  // compliant half space.
  props.AddProperty(kHydroGroup, kSlabThickness, 0.25);
  return props;
}

// Infrastructure to repeat tests on both double (and, someday, AutoDiffXd).
using ScalarTypes = ::testing::Types<double, AutoDiffXd>;

// Specification of the shape types to use with TestScene.
enum class ShapeType {
  kSphere,
  kHalfSpace
};

// Utility class for encoding a scene with two geometries. Each geometry can be
// either a sphere or a half space. The sphere gets tessellated and is
// representative of every shape type that is tessellated.
template <typename T>
class TestScene {
 public:
  explicit TestScene(ShapeType A_shape_type, ShapeType B_shape_type)
      : id_A_{GeometryId::get_new_id()},
        id_B_{GeometryId::get_new_id()},
        shape_A_type_(A_shape_type),
        shape_B_type_(B_shape_type),
        data_{&collision_filter_, &X_WGs_, &hydroelastic_geometries_,
              HydroelasticContactRepresentation::kTriangle,
              &surfaces_} {
    X_WGs_[id_A_] = RigidTransform<T>();
    X_WGs_[id_B_] = RigidTransform<T>();
  }

  // Configures a scene with two shapes (as specified in the constructor). Each
  // is given a hydroelastic representation based on the given requested types
  // and are positioned in a colliding state (if `are_colliding` is true).
  void ConfigureScene(const HydroelasticType type_A,
                      const HydroelasticType type_B,
                      bool are_colliding = true) {
    AddGeometry(type_A, type_B);
    PoseGeometry(are_colliding);
  }

  void AddGeometry(const HydroelasticType type_A,
                   const HydroelasticType type_B) {
    EncodedData data_A(id_A_, true);
    EncodedData data_B(id_B_, true);
    collision_filter_.AddGeometry(data_A.id());
    collision_filter_.AddGeometry(data_B.id());

    shape_A_ = MakeShape(id_A_, type_A, shape_A_type_, &data_A);
    shape_B_ = MakeShape(id_B_, type_B, shape_B_type_, &data_B);
  }

  // Adds a hydroelastic representation of the given `shape` with the given
  // compliance `type`.
  void MakeHydroelastic(GeometryId id, const HydroelasticType type,
                        const Shape& shape) {
    switch (type) {
      case HydroelasticType::kSoft:
        this->hydroelastic_geometries_.MaybeAddGeometry(shape, id,
                                                        soft_properties());
        break;
      case HydroelasticType::kRigid:
        this->hydroelastic_geometries_.MaybeAddGeometry(shape, id,
                                                        rigid_properties());
        break;
      case HydroelasticType::kUndefined:
        // Note: HydroelasticType::kUndefined will not be added.
        break;
    }
  }

  // Given the "description" of the shape to be added, does the work of
  // instantiating an FCL and hydroelastic representation. Returns the
  // fcl representation.
  unique_ptr<CollisionObjectd> MakeShape(GeometryId id, HydroelasticType type,
                                         ShapeType shape_type,
                                         EncodedData* data) {
    unique_ptr<CollisionObjectd> shape;
    switch (shape_type) {
      case ShapeType::kSphere:
        shape = make_unique<CollisionObjectd>(make_shared<Sphered>(kRadius));
        MakeHydroelastic(id, type, Sphere(kRadius));
        break;
      case ShapeType::kHalfSpace:
        shape = make_unique<CollisionObjectd>(make_shared<Halfspaced>());
        MakeHydroelastic(id, type, HalfSpace());
        break;
    }
    data->write_to(shape.get());
    return shape;
  }

  // Produce a rotation from a roll-pitch-yaw. If T = AutoDiffXd, the rotation
  // will have non-zero derivatives for roll, pitch, and yaw. The derivatives
  // will have size() == 3 and be *identical* for every RotationMatrix this
  // function creates.
  static RotationMatrix<T> MakeRotation(const Vector3d& rpy) {
    if constexpr (std::is_same_v<T, double>) {
      return RotationMatrix<T>(RollPitchYaw<T>(rpy));
    } else {
      const Vector3<T> rpy_ad = math::InitializeAutoDiff(rpy);
      return RotationMatrix<T>(RollPitchYaw<T>(rpy_ad));
    }
  }

  void PoseGeometry(bool are_colliding) {
    // If both shapes are half spaces, that is not contact we support and the
    // configuration is irrelevant (technically, their boundary planes could be
    // parallel and the half spaces could be disjoint -- but this test doesn't
    // care about that case).  Otherwise, the meaningful logic handles the case
    // where at least one is a sphere.
    //
    // If A is a sphere:
    //    - A stays at the origin.
    //    - B is moved _below_ shape A along the z axis an "appropriate"
    //      distance. The distance depends on:
    //      - the shape of geometry B and
    //      - whether collision is requested or not.
    // If A is a half space (and B is a sphere):
    //    - A's orientation is reversed, so that its outward normal points in
    //      the -Wz direction.
    //    - B is still placed "below" A based on the same kind of logic.
    //
    // For detecting "successful" differentiation, we define R_WA and R_WB as
    // having derivatives w.r.t. some arbitrary quantities. That way, the
    // transformation of both points and vectors will accumulate derivatives.

    if (this->shape_A_type_ == ShapeType::kHalfSpace &&
        this->shape_B_type_ == ShapeType::kHalfSpace)
      return;

    // The distance from the world origin to the surface of shape A. In other
    // words, a point at Wo needs to be moved `distance` units to lie on the
    // surface of A.
    const double dist_WoA =
        this->shape_A_type_ == ShapeType::kSphere ? kRadius : 0;
    const Vector3d rpy_WA{M_PI, 0, 0};

    // Remember that every AutoDiffXd-valued RotationMatrix we make has
    // derivatives with respect to three arbitrary, magical quantities. That is
    // why we only expect the results to have derivatives.size() == 3.
    X_WGs_[id_A_] =
        RigidTransform<T>(MakeRotation(rpy_WA), Vector3<T>(0, 0, 0));

    // Similarly, the distance to the surface of B (see dist_WoA).
    const double dist_WoB =
        this->shape_B_type_ == ShapeType::kSphere ? kRadius : 0;
    const double z_offset =
        -(dist_WoA + dist_WoB) + (are_colliding ? 0.125 : -0.125);
    const Vector3d p_WB{0, 0, z_offset};
    const Vector3d rpy_WB{0, 0, 0};
    X_WGs_[id_B_] = RigidTransform<T>(MakeRotation(rpy_WB), p_WB.cast<T>());
  }

  // Filters contact between the two spheres.
  void FilterContact() {
    EncodedData data_A(*shape_A_);
    EncodedData data_B(*shape_B_);
    // Filter the pair (A, B); we'll put the ids in a set and simply return that
    // set for the extract ids function.
    std::unordered_set<GeometryId> ids{data_A.id(), data_B.id()};
    CollisionFilter::ExtractIds extract = [&ids](const GeometrySet&) {
      return ids;
    };
    collision_filter_.Apply(CollisionFilterDeclaration().ExcludeWithin(
                                GeometrySet{data_A.id(), data_B.id()}),
                            extract, false /* is_invariant */);
  }

  // Note: these are non const because the callback takes non-const pointers
  // (due to FCL's API).
  CollisionObjectd& shape_A() { return *shape_A_; }
  CollisionObjectd& shape_B() { return *shape_B_; }
  CallbackData<T>& data() { return data_; }
  const vector<ContactSurface<T>>& surfaces() const { return surfaces_; }
  const Geometries& hydroelastic_geometries() const {
    return hydroelastic_geometries_;
  }
  GeometryId id_A() const { return id_A_; }
  GeometryId id_B() const { return id_B_; }
  const RigidTransform<T>& pose_in_world(GeometryId id) const {
    return X_WGs_.at(id);
  }

 private:
  Geometries hydroelastic_geometries_;
  CollisionFilter collision_filter_;
  unordered_map<GeometryId, RigidTransform<T>> X_WGs_;
  GeometryId id_A_{};
  GeometryId id_B_{};
  static constexpr double kRadius{0.25};
  const ShapeType shape_A_type_;
  const ShapeType shape_B_type_;
  unique_ptr<CollisionObjectd> shape_A_;
  unique_ptr<CollisionObjectd> shape_B_;
  vector<ContactSurface<T>> surfaces_;
  CallbackData<T> data_;
};

// All double-valued contact surfaces have "valid" derivatives (aka none).
::testing::AssertionResult ValidateDerivatives(const ContactSurface<double>&) {
  return ::testing::AssertionSuccess();
}

// For an AutoDiffXd-valued surface, we rely on the fact that the scene has been
// configured to be differentiated with respect to three unknown quantities.
// And those derivatives are introduced via rpy_WA and rpy_WB. So, any quantity
// that ultimately depends on the relative orientation of those bases should
// exhibit derivatives with size() == 3. In this case, *all* of these quantities
// depend on the relative orientation of bases of A and B. So, we poke at a
// couple of different quantities to make sure they have appropriately *sized*
// derivatives. We don't worry about the *values* here, relying on other code to
// have already tested that. This isn't an *exhaustive* test at all; it's merely
// looking for positive indicators that the rigorously tested intersection code
// has been properly connected in the callback.
//
// Only one of grad e_N or grad e_M will be defined -- it is the field from the
// soft mesh.
::testing::AssertionResult ValidateDerivatives(
    const ContactSurface<AutoDiffXd>& surface) {
  if (surface.representation() ==
      HydroelasticContactRepresentation::kTriangle) {
    const auto& mesh_W = surface.tri_mesh_W();
    if (mesh_W.vertex(0).x().derivatives().size() != 3) {
      return ::testing::AssertionFailure() << "Vertex 0 is missing derivatives";
    }

    if (surface.tri_e_MN().EvaluateAtVertex(0).derivatives().size() != 3) {
      return ::testing::AssertionFailure()
             << "Pressure field at vertex 0 is missing derivatives";
    }
  } else {
    const auto& mesh_W = surface.poly_mesh_W();
    if (mesh_W.vertex(0).x().derivatives().size() != 3) {
      return ::testing::AssertionFailure() << "Vertex 0 is missing derivatives";
    }

    if (surface.poly_e_MN().EvaluateAtVertex(0).derivatives().size() != 3) {
      return ::testing::AssertionFailure()
             << "Pressure field at vertex 0 is missing derivatives";
    }
  }

  if (surface.HasGradE_M()) {
    if (surface.EvaluateGradE_M_W(0).x().derivatives().size() != 3) {
      return ::testing::AssertionFailure()
             << "Face 0's grad eM is missing derivatives";
    }
  }
  if (surface.HasGradE_N()) {
    if (surface.EvaluateGradE_N_W(0).x().derivatives().size() != 3) {
      return ::testing::AssertionFailure()
             << "Face 0's grad eN is missing derivatives";
    }
  }

  return ::testing::AssertionSuccess();
}

TYPED_TEST_SUITE(DispatchRigidSoftCalculationTests, ScalarTypes);

// Test suite for exercising DispatchRigidSoftCalculation with different scalar
// types. (Currently only double as the hydroelastic infrastructure doesn't
// support autodiff yet.)
template <typename T>
class DispatchRigidSoftCalculationTests : public ::testing::Test {};

// Tests that a pair of meshes (soft and rigid) gets properly dispatched.
TYPED_TEST(DispatchRigidSoftCalculationTests, SoftMeshRigidMesh) {
  using T = TypeParam;
  const bool colliding{true};

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kSphere};
  const Geometries& geometries = scene.hydroelastic_geometries();
  const GeometryId id_A = scene.id_A();
  const GeometryId id_B = scene.id_B();
  // Sphere A has a fixed position.
  const RigidTransform<T>& X_WA = scene.pose_in_world(id_A);
  scene.AddGeometry(HydroelasticType::kSoft, HydroelasticType::kRigid);

  for (const auto representation :
       {HydroelasticContactRepresentation::kTriangle,
        HydroelasticContactRepresentation::kPolygon}) {
    SCOPED_TRACE(fmt::format("representation = {}", representation));
    {
      // Case 1: Intersecting spheres.
      scene.PoseGeometry(colliding);
      const RigidTransform<T>& X_WB = scene.pose_in_world(id_B);

      unique_ptr<ContactSurface<T>> surface = DispatchRigidSoftCalculation(
          geometries.soft_geometry(id_A), X_WA, id_A,
          geometries.rigid_geometry(id_B), X_WB, id_B, representation);
      EXPECT_NE(surface, nullptr);
      EXPECT_TRUE(ValidateDerivatives(*surface));
      switch (representation) {
        case HydroelasticContactRepresentation::kTriangle:
          EXPECT_EQ(100, surface->tri_mesh_W().num_triangles());
          break;
        case HydroelasticContactRepresentation::kPolygon:
          EXPECT_EQ(28, surface->poly_mesh_W().num_elements());
          break;
      }
    }
  }

  {
    // Case 2: Separated spheres.
    scene.PoseGeometry(!colliding);
    const RigidTransform<T>& X_WB = scene.pose_in_world(id_B);

    // When they are not in contact, mesh representation is irrelevant; so,
    // we'll simply pick one.
    unique_ptr<ContactSurface<T>> surface = DispatchRigidSoftCalculation(
        geometries.soft_geometry(id_A), X_WA, id_A,
        geometries.rigid_geometry(id_B), X_WB, id_B,
        HydroelasticContactRepresentation::kTriangle);
    EXPECT_EQ(surface, nullptr);
  }
}

// Tests that a rigid half space and soft mesh gets properly dispatched.
TYPED_TEST(DispatchRigidSoftCalculationTests, SoftMeshRigidHalfSpace) {
  using T = TypeParam;
  const bool colliding{true};

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kHalfSpace};
  const Geometries& geometries = scene.hydroelastic_geometries();
  const GeometryId id_A = scene.id_A();
  const GeometryId id_B = scene.id_B();
  // Sphere A has a fixed position.
  const RigidTransform<T>& X_WA = scene.pose_in_world(id_A);
  scene.AddGeometry(HydroelasticType::kSoft, HydroelasticType::kRigid);

  for (const auto representation :
       {HydroelasticContactRepresentation::kTriangle,
        HydroelasticContactRepresentation::kPolygon}) {
    SCOPED_TRACE(fmt::format("representation = {}", representation));
    // Case 1: Intersecting geometry.
    scene.PoseGeometry(colliding);
    const RigidTransform<T>& X_WB = scene.pose_in_world(id_B);

    unique_ptr<ContactSurface<T>> surface = DispatchRigidSoftCalculation(
        geometries.soft_geometry(id_A), X_WA, id_A,
        geometries.rigid_geometry(id_B), X_WB, id_B, representation);
    EXPECT_NE(surface, nullptr);
    EXPECT_TRUE(ValidateDerivatives(*surface));
  }

  {
    // Case 2: Separated geometry.
    scene.PoseGeometry(!colliding);
    const RigidTransform<T>& X_WB = scene.pose_in_world(id_B);

    // When they are not in contact, mesh representation is irrelevant; so,
    // we'll simply pick one.
    unique_ptr<ContactSurface<T>> surface = DispatchRigidSoftCalculation(
        geometries.soft_geometry(id_A), X_WA, id_A,
        geometries.rigid_geometry(id_B), X_WB, id_B,
        HydroelasticContactRepresentation::kTriangle);
    EXPECT_EQ(surface, nullptr);
  }
}

TYPED_TEST(DispatchRigidSoftCalculationTests, SoftHalfSpaceRigidMesh) {
  using T = TypeParam;
  const bool colliding{true};

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kHalfSpace};
  const Geometries& geometries = scene.hydroelastic_geometries();
  const GeometryId id_A = scene.id_A();
  const GeometryId id_B = scene.id_B();
  // Sphere A has a fixed position.
  const RigidTransform<T>& X_WA = scene.pose_in_world(id_A);
  scene.AddGeometry(HydroelasticType::kRigid, HydroelasticType::kSoft);

  for (const auto representation :
       {HydroelasticContactRepresentation::kTriangle,
        HydroelasticContactRepresentation::kPolygon}) {
    SCOPED_TRACE(fmt::format("representation = {}", representation));
    // Case 1: Intersecting geometry.
    scene.PoseGeometry(colliding);
    const RigidTransform<T>& X_WB = scene.pose_in_world(id_B);

    unique_ptr<ContactSurface<T>> surface = DispatchRigidSoftCalculation(
        geometries.soft_geometry(id_B), X_WB, id_A,
        geometries.rigid_geometry(id_A), X_WA, id_B, representation);
    EXPECT_NE(surface, nullptr);
    EXPECT_TRUE(ValidateDerivatives(*surface));
  }

  {
    // Case 2: Separated geometry.
    scene.PoseGeometry(!colliding);
    const RigidTransform<T>& X_WB = scene.pose_in_world(id_B);

    // When they are not in contact, mesh representation is irrelevant; so,
    // we'll simply pick one.
    unique_ptr<ContactSurface<T>> surface = DispatchRigidSoftCalculation(
        geometries.soft_geometry(id_B), X_WB, id_A,
        geometries.rigid_geometry(id_A), X_WA, id_B,
        HydroelasticContactRepresentation::kTriangle);
    EXPECT_EQ(surface, nullptr);
  }
}

TYPED_TEST_SUITE(MaybeCalcContactSurfaceTests, ScalarTypes);

// Test suite for exercising MaybeCalcContactSurface with different scalar
// types. (Currently only double as the hydroelastic infrastructure doesn't
// support autodiff yet.)
template <typename T>
class MaybeCalcContactSurfaceTests : public ::testing::Test {};

// Confirms that if one or both geometries do not have a hydroelastic
// representation, that the proper result is returned.
TYPED_TEST(MaybeCalcContactSurfaceTests, UndefinedGeometry) {
  using T = TypeParam;

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kSphere};
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kUndefined);

  // Case: second is undefined.
  CalcContactSurfaceResult result = MaybeCalcContactSurface<T>(
      &scene.shape_A(), &scene.shape_B(), &scene.data());
  ASSERT_EQ(result, CalcContactSurfaceResult::kUnsupported);
  ASSERT_EQ(scene.surfaces().size(), 0u);

  // Case: first is undefined.
  result = MaybeCalcContactSurface<T>(
      &scene.shape_B(), &scene.shape_A(), &scene.data());
  ASSERT_EQ(result, CalcContactSurfaceResult::kUnsupported);
  ASSERT_EQ(scene.surfaces().size(), 0u);

  // Case: both are undefined.
  result = MaybeCalcContactSurface<T>(
      &scene.shape_B(), &scene.shape_B(), &scene.data());
  ASSERT_EQ(result, CalcContactSurfaceResult::kUnsupported);
  ASSERT_EQ(scene.surfaces().size(), 0u);
}

// Confirms that matching compliance (rigid) can't be evaluated.
TYPED_TEST(MaybeCalcContactSurfaceTests, BothRigid) {
  using T = TypeParam;

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kSphere};
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kRigid);

  CalcContactSurfaceResult result = MaybeCalcContactSurface<T>(
      &scene.shape_A(), &scene.shape_B(), &scene.data());
  EXPECT_EQ(result, CalcContactSurfaceResult::kSameCompliance);
  EXPECT_EQ(scene.surfaces().size(), 0u);
}

// Confirms that matching compliance (soft) can't be evaluated.
TYPED_TEST(MaybeCalcContactSurfaceTests, BothSoft) {
  using T = TypeParam;

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kSphere};
  scene.ConfigureScene(HydroelasticType::kSoft, HydroelasticType::kSoft);

  CalcContactSurfaceResult result = MaybeCalcContactSurface<T>(
      &scene.shape_A(), &scene.shape_B(), &scene.data());
  EXPECT_EQ(result, CalcContactSurfaceResult::kSameCompliance);
  EXPECT_EQ(scene.surfaces().size(), 0u);
}

// Confirms that colliding two half spaces is detected and reported.
TYPED_TEST(MaybeCalcContactSurfaceTests, TwoHalfSpaces) {
  using T = TypeParam;

  TestScene<T> scene{ShapeType::kHalfSpace, ShapeType::kHalfSpace};
  // They must have different compliance types in order to get past the same
  // compliance type condition.
  scene.ConfigureScene(HydroelasticType::kSoft, HydroelasticType::kRigid,
                       false /* are_colliding */);

  CalcContactSurfaceResult result = MaybeCalcContactSurface<T>(
      &scene.shape_A(), &scene.shape_B(), &scene.data());
  EXPECT_EQ(result, CalcContactSurfaceResult::kHalfSpaceHalfSpace);
  EXPECT_EQ(scene.surfaces().size(), 0u);
}

// Confirms that a valid pair that are, nevertheless, not colliding does not
// add a surface to the results.
TYPED_TEST(MaybeCalcContactSurfaceTests, NonColliding) {
  using T = TypeParam;

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kSphere};
  scene.ConfigureScene(HydroelasticType::kSoft, HydroelasticType::kRigid,
                       false /* are_colliding */);

  CalcContactSurfaceResult result = MaybeCalcContactSurface<T>(
      &scene.shape_A(), &scene.shape_B(), &scene.data());
  EXPECT_EQ(result, CalcContactSurfaceResult::kCalculated);
  EXPECT_EQ(scene.surfaces().size(), 0u);
}

// Confirms that a valid pair of hydroelastic representations report as such
// and produce a result. (The details of the result are not explicitly
// evaluated as they have been tested by the underlying method's unit tests.)
TYPED_TEST(MaybeCalcContactSurfaceTests, HandleSoftMeshRigidMesh) {
  using T = TypeParam;

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kSphere};
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kSoft,
                       true /* are_colliding */);

  CalcContactSurfaceResult result = MaybeCalcContactSurface<T>(
      &scene.shape_A(), &scene.shape_B(), &scene.data());
  EXPECT_EQ(result, CalcContactSurfaceResult::kCalculated);
  EXPECT_EQ(scene.surfaces().size(), 1u);
  EXPECT_TRUE(ValidateDerivatives(scene.surfaces()[0]));
}

// Confirms that MaybeCalcContactSurface relies on DispatchRigidSoftCalculation
// as indicated (but not proven) by its ability to handle mesh-half space
// contact.
TYPED_TEST(MaybeCalcContactSurfaceTests, HandleSoftMeshRigidHalfspace) {
  using T = TypeParam;

  TestScene<T> scene(ShapeType::kSphere, ShapeType::kHalfSpace);
  scene.ConfigureScene(HydroelasticType::kSoft, HydroelasticType::kRigid);
  CalcContactSurfaceResult result = MaybeCalcContactSurface<T>(
      &scene.shape_A(), &scene.shape_B(), &scene.data());
  EXPECT_EQ(result, CalcContactSurfaceResult::kCalculated);
  EXPECT_EQ(scene.surfaces().size(), 1u);
  EXPECT_TRUE(ValidateDerivatives(scene.surfaces()[0]));
}

TYPED_TEST_SUITE(StrictHydroelasticCallbackTyped, ScalarTypes);

// Test infrastructure for the strict hydroelastic callback for arbitrary
// scalar type. It makes use of MaybeCalculationContactSurface() but this method
// has the following responsibilities:
//   - invoke the method using the data provided to it (so that the results
//     ultimately percolate outward).
//   - throw on any return value that isn't wholly successful.
//   - respect collision filtering.
// (Currently only double as the hydroelastic infrastructure doesn't support
// autodiff yet.)
template <typename T>
class StrictHydroelasticCallbackTyped : public ::testing::Test {};

// Confirms that if the intersecting pair is missing hydroelastic representation
// that an exception is thrown. This test applies *no* collision filters
// to guarantee that the body of the callback gets exercised in all cases.
TYPED_TEST(StrictHydroelasticCallbackTyped,
           ThrowForMissingHydroelasticRepresentation) {
  using T = TypeParam;

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kSphere};
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kUndefined);

  // We test only a single "underrepresented" configuration (rigid, undefined)
  // because we rely on the tests on MaybeCalcContactSurface() to have explored
  // all the ways that the kUnsupported calculation result is returned. This
  // configuration is representative of that set.
  DRAKE_EXPECT_THROWS_MESSAGE(
      Callback<T>(&scene.shape_A(), &scene.shape_B(), &scene.data()),
      std::logic_error,
      "Requested a contact surface between a pair of geometries without "
      "hydroelastic representation .+ rigid .+ undefined .+");
}

// Confirms that if the intersecting pair has the same compliance (rigid-rigid)
// or (soft-soft), that an exception is thrown. This test applies *no* collision
// filters to guarantee that the body of the callback gets exercised in all
// cases.
TYPED_TEST(StrictHydroelasticCallbackTyped, ThrowForSameComplianceType) {
  using T = TypeParam;

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kSphere};
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kRigid);

  // We test only a single "same-compliance" configuration (rigid, rigid)
  // because we rely on the tests on MaybeCalcContactSurface() to have explored
  // all the ways that the kSameCompliance calculation result is returned. This
  // configuration is representative of that set.
  DRAKE_EXPECT_THROWS_MESSAGE(
      Callback<T>(&scene.shape_A(), &scene.shape_B(), &scene.data()),
      std::logic_error,
      "Requested contact between two rigid objects .+");
}

// Confirms that if the intersecting pair consists of two half spaces that an
// exception is thrown.
TYPED_TEST(StrictHydroelasticCallbackTyped, ThrowForTwoHalfSpaces) {
  using T = TypeParam;

  TestScene<T> scene{ShapeType::kHalfSpace, ShapeType::kHalfSpace};
  // They must have different compliance types in order to get past the same
  // compliance type condition.
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kSoft);

  DRAKE_EXPECT_THROWS_MESSAGE(
      Callback<T>(&scene.shape_A(), &scene.shape_B(), &scene.data()),
      std::logic_error,
      "Requested contact between two half spaces .+");
}

// Confirms that if the pair contains unsupported geometry, as long as they are
// filtered, they don't pose a problem. The "lack of support" comes from the
// fact that the ids don't map to anything in the hydroelastic geometry set.
TYPED_TEST(StrictHydroelasticCallbackTyped, RespectsCollisionFilter) {
  using T = TypeParam;

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kSphere};
  // Note: a configuration that would cause an exception to be thrown if
  // unfiltered and the confirmation of that assumption.
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kRigid);
  EXPECT_THROW(Callback<T>(&scene.shape_A(), &scene.shape_B(), &scene.data()),
               std::logic_error);

  scene.FilterContact();
  DRAKE_EXPECT_NO_THROW(
      Callback<T>(&scene.shape_A(), &scene.shape_B(), &scene.data()));
  EXPECT_EQ(scene.surfaces().size(), 0u);
}

// Confirms that a colliding collision pair (with supported hydroelastic
// representations) produces a result. This doesn't test the actual data -- it
// assumes the function responsible for computing that result has been
// successfully tested. This test is subtle; it simply confirms that the
// Callback invokes MaybeCalcContactSurface() and provides the correct
// vector<ContactSurface> instance.
TYPED_TEST(StrictHydroelasticCallbackTyped, ValidPairProducesResult) {
  using T = TypeParam;

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kSphere};
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kSoft);

  DRAKE_EXPECT_NO_THROW(
      Callback<T>(&scene.shape_A(), &scene.shape_B(), &scene.data()));
  EXPECT_EQ(scene.surfaces().size(), 1u);
  EXPECT_TRUE(ValidateDerivatives(scene.surfaces()[0]));
}

TYPED_TEST_SUITE(HydroelasticCallbackFallbackTyped, ScalarTypes);

// Test infrastructure for the hydroelastic callback with fallback for arbitrary
// scalar type. It makes use of MaybeCalculationContactSurface() but this method
// has the following responsibilities:
//   - invoke the method using the data provided to it (so that the results
//     ultimately percolate outward).
//   - Compute point pair penetration for any pair that couldn't be calculated.
//   - respect collision filtering.
// (Currently only double as the hydroelastic infrastructure doesn't support
// autodiff yet.)
// The tests below parallel the tests for the strict hydroelastic callback.
template <typename T>
class HydroelasticCallbackFallbackTyped : public ::testing::Test {};

// Confirms that if the intersecting pair is missing hydroelastic representation
// a point pair is generated. This test applies *no* collision filters
// to guarantee that the body of the callback gets exercised in all cases.
TYPED_TEST(HydroelasticCallbackFallbackTyped,
           PointPairForMissingHydroelasticRepresentation) {
  using T = TypeParam;

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kSphere};
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kUndefined);

  // We test only a single "underrepresented" configuration (rigid, undefined)
  // because we rely on the tests on MaybeCalcContactSurface() to have explored
  // all the ways that the kUnsupported calculation result is returned. This
  // configuration is representative of that set.
  vector<PenetrationAsPointPair<T>> point_pairs;
  CallbackWithFallbackData<T> data{scene.data(), &point_pairs};
  DRAKE_EXPECT_NO_THROW(
      CallbackWithFallback<T>(&scene.shape_A(), &scene.shape_B(), &data));
  EXPECT_EQ(scene.surfaces().size(), 0u);
  EXPECT_EQ(point_pairs.size(), 1u);
}

// Confirms that if the intersecting pair has the same compliance (rigid-rigid)
// or (soft-soft), that we return a point-pair. This test applies *no* collision
// filters to guarantee that the body of the callback gets exercised in all
// cases.
TYPED_TEST(HydroelasticCallbackFallbackTyped, PointPairForSameComplianceType) {
  using T = TypeParam;

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kSphere};
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kRigid);

  // We test only a single "same-compliance" configuration (rigid, rigid)
  // because we rely on the tests on MaybeCalcContactSurface() to have explored
  // all the ways that the kSameCompliance calculation result is returned. This
  // configuration is representative of that set.
  vector<PenetrationAsPointPair<T>> point_pairs;
  CallbackWithFallbackData<T> data{scene.data(), &point_pairs};
  DRAKE_EXPECT_NO_THROW(
      CallbackWithFallback<T>(&scene.shape_A(), &scene.shape_B(), &data));
  EXPECT_EQ(scene.surfaces().size(), 0u);
  EXPECT_EQ(point_pairs.size(), 1u);
}

// Confirms that collision filters are respected; for a filtered colliding pair,
// no results are returned at all.
TYPED_TEST(HydroelasticCallbackFallbackTyped, RespectsCollisionFilter) {
  using T = TypeParam;

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kSphere};
  // Note: a configuration that would cause an exception to be thrown if
  // unfiltered and the confirmation of that assumption.
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kRigid);

  vector<PenetrationAsPointPair<T>> point_pairs;
  CallbackWithFallbackData<T> data{scene.data(), &point_pairs};
  // Confirm collision state.
  DRAKE_EXPECT_NO_THROW(
      CallbackWithFallback<T>(&scene.shape_A(), &scene.shape_B(), &data));
  EXPECT_EQ(scene.surfaces().size(), 0u);
  EXPECT_EQ(point_pairs.size(), 1u);

  // Now filter.
  point_pairs.clear();
  scene.FilterContact();

  DRAKE_EXPECT_NO_THROW(
      CallbackWithFallback<T>(&scene.shape_A(), &scene.shape_B(), &data));
  EXPECT_EQ(scene.surfaces().size(), 0u);
  EXPECT_EQ(point_pairs.size(), 0u);
}

// Confirms that a colliding collision pair (with supported hydroelastic
// representations) produces a contact surface. This doesn't test the actual
// data -- it assumes the function responsible for computing that result has
// been successfully tested. This test is subtle; it simply confirms that the
// Callback invokes MaybeCalcContactSurface() and provides the correct
// vector<ContactSurface> instance.
TYPED_TEST(HydroelasticCallbackFallbackTyped, ValidPairProducesResult) {
  using T = TypeParam;

  TestScene<T> scene{ShapeType::kSphere, ShapeType::kSphere};
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kSoft);

  vector<PenetrationAsPointPair<T>> point_pairs;
  CallbackWithFallbackData<T> data{scene.data(), &point_pairs};

  DRAKE_EXPECT_NO_THROW(
      CallbackWithFallback<T>(&scene.shape_A(), &scene.shape_B(), &data));
  EXPECT_EQ(scene.surfaces().size(), 1u);
  EXPECT_TRUE(ValidateDerivatives(scene.surfaces()[0]));
  EXPECT_EQ(point_pairs.size(), 0u);
}

}  // namespace
}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake

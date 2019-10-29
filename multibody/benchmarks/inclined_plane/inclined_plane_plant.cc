#include "drake/multibody/benchmarks/inclined_plane/inclined_plane_plant.h"

#include <string>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace inclined_plane {

using math::RigidTransform;
using math::RotationMatrix;

void AddInclinedPlaneAndGravityToPlant(
    double gravity, double inclined_plane_angle,
    const std::optional<Vector3<double>>& inclined_plane_dimensions,
    const CoulombFriction<double>& coefficient_friction_inclined_plane,
    MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);

  // See the documentation for the orientation of Ax, Ay, Az and Wx, Wy, Wz.
  const RotationMatrix<double> R_WA =
      RotationMatrix<double>::MakeYRotation(inclined_plane_angle);

  // The inclined plane A is either a half-space or a box.
  const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
  if (inclined_plane_dimensions == std::nullopt) {
    // Set A's geometry so its top surface passes through world origin Wo.
    const Vector3<double> p_WoAo_W = Vector3<double>::Zero();
    const RigidTransform<double> X_WA(R_WA, p_WoAo_W);
    plant->RegisterVisualGeometry(plant->world_body(), X_WA,
                                  geometry::HalfSpace(),
                                  "InclinedPlaneVisualGeometry",
                                  green);
    plant->RegisterCollisionGeometry(plant->world_body(), X_WA,
                                     geometry::HalfSpace(),
                                     "InclinedPlaneCollisionGeometry",
                                     coefficient_friction_inclined_plane);
  } else {
    // If using a box, ensure its dimensions are positive.
    const double LAx = inclined_plane_dimensions->x();
    const double LAy = inclined_plane_dimensions->y();
    const double LAz = inclined_plane_dimensions->z();
    DRAKE_THROW_UNLESS(LAx > 0 && LAy > 0 && LAz > 0);

    // Set inclined plane A's visual geometry and collision geometry to a
    // box whose top surface passes through world origin Wo.  To do this,
    // set Ao's position from Wo as -0.5 * LAz * Az (half-width of box A).
    const Vector3<double> Az_W = R_WA.col(2);  // Az in terms of Wx, Wy, Wz.
    const Vector3<double> p_WoAo_W = -0.5 * LAz * Az_W;
    const RigidTransform<double> X_WA(R_WA, p_WoAo_W);
    plant->RegisterVisualGeometry(plant->world_body(), X_WA,
                                  geometry::Box(LAx, LAy, LAz),
                                  "InclinedPlaneVisualGeometry",
                                  green);
    plant->RegisterCollisionGeometry(plant->world_body(), X_WA,
                                     geometry::Box(LAx, LAy, LAz),
                                     "InclinedPlaneCollisionGeometry",
                                     coefficient_friction_inclined_plane);
  }

  // Earth's local gravity acts in the world's -z direction.
  const Vector3<double> gravity_vector_W(0, 0, -gravity);
  plant->mutable_gravity_field().set_gravity_vector(gravity_vector_W);
}

void AddInclinedPlaneWithBlockToPlant(
    double gravity, double inclined_plane_angle,
    const std::optional<Vector3<double>>& inclined_plane_dimensions,
    const CoulombFriction<double>& coefficient_friction_inclined_plane,
    const CoulombFriction<double>& coefficient_friction_bodyB,
    double massB, const Vector3<double>& block_dimensions,
    bool is_block_with_4Spheres, MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);

  AddInclinedPlaneAndGravityToPlant(
      gravity, inclined_plane_angle, inclined_plane_dimensions,
      coefficient_friction_inclined_plane, plant);

  // Ensure the block's dimensions are mass are positive.
  const double LBx = block_dimensions.x();
  const double LBy = block_dimensions.y();
  const double LBz = block_dimensions.z();
  DRAKE_THROW_UNLESS(LBx > 0 && LBy > 0 && LBz > 0 && massB > 0);

  // Describe body B's mass, center of mass, and inertia properties.
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
  const UnitInertia<double> G_BBcm_B =
      UnitInertia<double>::SolidBox(LBx, LBy, LBz);
  const SpatialInertia<double> M_BBcm_B(massB, p_BoBcm_B, G_BBcm_B);

  // Create a rigid body B with the mass properties of a uniform solid block.
  const RigidBody<double>& blockB = plant->AddRigidBody("BodyB", M_BBcm_B);

  // Body B's visual geometry is always a solid box.
  // The pose X_BG of block B's geometry frame G is an identity transform.
  const RigidTransform<double> X_BG;   // Identity transform.
  const Vector4<double> lightBlue(0.5, 0.8, 1.0, 1.0);
  plant->RegisterVisualGeometry(blockB, X_BG,
                                geometry::Box(LBx, LBy, LBz),
                                "BlockB_VisualGeometry", lightBlue);

  // Block B's contacting surface is either a box or 4 spheres.
  if (is_block_with_4Spheres) {
    // There are four red spheres welded to the block B's bottom surface.
    // The rotation matrix for each of these spheres is the 3x3 identity matrix.
    const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
    const double radius = LBz/2;
    int i = 0;
    for (double x_sign : {-1.0, 1.0}) {
      for (double y_sign : {-1.0, 1.0}) {
        const std::string name_spherei =
            "Sphere" + std::to_string(++i) + "_Geometry";
        const double x = x_sign * LBx / 2;
        const double y = y_sign * LBy / 2;
        const double z = -radius;
        const Vector3<double> p_BoSpherei_B(x, y, z);
        const RigidTransform<double> X_BSpherei(p_BoSpherei_B);
        plant->RegisterCollisionGeometry(blockB, X_BSpherei,
                                         geometry::Sphere(radius),
                                         name_spherei,
                                         coefficient_friction_bodyB);
        plant->RegisterVisualGeometry(blockB, X_BSpherei,
                                      geometry::Sphere(radius),
                                      name_spherei, red);
      }
    }
  } else {
    // Block B's collision geometry is a solid box.
    plant->RegisterCollisionGeometry(blockB, X_BG,
                                     geometry::Box(LBx, LBy, LBz),
                                     "BlockB_VisualGeometry",
                                     coefficient_friction_bodyB);
  }
}

void AddInclinedPlaneWithSphereToPlant(
    double gravity, double inclined_plane_angle,
    const std::optional<Vector3<double>>& inclined_plane_dimensions,
    const CoulombFriction<double>& coefficient_friction_inclined_plane,
    const CoulombFriction<double>& coefficient_friction_bodyB,
    double massB, double radiusB, MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(radiusB > 0 && massB > 0);

  AddInclinedPlaneAndGravityToPlant(
      gravity, inclined_plane_angle, inclined_plane_dimensions,
      coefficient_friction_inclined_plane, plant);

  // Describe body B's mass, center of mass, and inertia properties.
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
  const UnitInertia<double> G_BBcm = UnitInertia<double>::SolidSphere(radiusB);
  const SpatialInertia<double> M_BBcm_B(massB, p_BoBcm_B, G_BBcm);

  // Create a rigid body B with the mass properties of a uniform sphere.
  const RigidBody<double>& sphereB = plant->AddRigidBody("BodyB", M_BBcm_B);

  // Body B's visual geometry and collision geometry are a sphere.
  // The pose X_BG of block B's geometry frame G is an identity transform.
  const RigidTransform<double> X_BG;   // Identity transform.
  const Vector4<double> lightBlue(0.5, 0.8, 1.0, 1.0);
  plant->RegisterVisualGeometry(sphereB, X_BG,
                                geometry::Sphere(radiusB),
                                "SphereB_VisualGeometry", lightBlue);
  plant->RegisterCollisionGeometry(sphereB, X_BG,
                                   geometry::Sphere(radiusB),
                                   "SphereB_CollisionGeometry",
                                   coefficient_friction_bodyB);

  // Add little spherical spokes to highlight the sphere's rotation.
  const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
  const double radius_spoke = radiusB / 5;
  const RigidTransform<double> X_B_Spoke1(Vector3<double>(0, 0, radiusB));
  const RigidTransform<double> X_B_Spoke2(Vector3<double>(0, 0, -radiusB));
  const RigidTransform<double> X_B_Spoke3(Vector3<double>(radiusB, 0, 0));
  const RigidTransform<double> X_B_Spoke4(Vector3<double>(-radiusB, 0, 0));
  plant->RegisterVisualGeometry(sphereB, X_B_Spoke1,
                                geometry::Sphere(radius_spoke),
                                "Spoke1_VisualGeometry", red);
  plant->RegisterVisualGeometry(sphereB, X_B_Spoke2,
                                geometry::Sphere(radius_spoke),
                                "Spoke2_VisualGeometry", red);
  plant->RegisterVisualGeometry(sphereB, X_B_Spoke3,
                                geometry::Sphere(radius_spoke),
                                "Spoke3_VisualGeometry", red);
  plant->RegisterVisualGeometry(sphereB, X_B_Spoke4,
                                geometry::Sphere(radius_spoke),
                                "Spoke4_VisualGeometry", red);
}

}  // namespace inclined_plane
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake

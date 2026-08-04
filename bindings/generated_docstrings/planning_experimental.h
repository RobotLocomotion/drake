#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/planning/experimental/mbp_environment_collision_checker.h"
// #include "drake/planning/experimental/placeholder.h"
// #include "drake/planning/experimental/sphere_robot_model_collision_checker.h"
// #include "drake/planning/experimental/voxel_occupancy_map.h"
// #include "drake/planning/experimental/voxel_self_filter.h"
// #include "drake/planning/experimental/voxel_signed_distance_field.h"
// #include "drake/planning/experimental/voxel_tagged_object_occupancy_map.h"
// #include "drake/planning/experimental/voxelized_environment_builder.h"
// #include "drake/planning/experimental/voxelized_environment_collision_checker.h"

// Symbol: pydrake_doc_planning_experimental
constexpr struct /* pydrake_doc_planning_experimental */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::planning
    struct /* planning */ {
      // Symbol: drake::planning::experimental
      struct /* experimental */ {
        // Symbol: drake::planning::experimental::BodySpheres
        struct /* BodySpheres */ {
          // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
          const char* doc =
R"""(Container for all spheres belonging to a single body's collision
model. API is generally equivalent to that of map<GeometryId,
SphereSpecification>. Interator support is provided to enable easy
iteration over the spheres.)""";
          // Symbol: drake::planning::experimental::BodySpheres::At
          struct /* At */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } At;
          // Symbol: drake::planning::experimental::BodySpheres::BodySpheres
          struct /* ctor */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::planning::experimental::BodySpheres::Empty
          struct /* Empty */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } Empty;
          // Symbol: drake::planning::experimental::BodySpheres::Size
          struct /* Size */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } Size;
          // Symbol: drake::planning::experimental::BodySpheres::begin
          struct /* begin */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } begin;
          // Symbol: drake::planning::experimental::BodySpheres::bounding_sphere
          struct /* bounding_sphere */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc =
R"""(Retrieve the bounding sphere, if available, or throws if the bounding
sphere is not available.)""";
          } bounding_sphere;
          // Symbol: drake::planning::experimental::BodySpheres::const_iterator
          struct /* const_iterator */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } const_iterator;
          // Symbol: drake::planning::experimental::BodySpheres::end
          struct /* end */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } end;
        } BodySpheres;
        // Symbol: drake::planning::experimental::BuildOccupancyMap
        struct /* BuildOccupancyMap */ {
          // Source: drake/planning/experimental/voxelized_environment_builder.h
          const char* doc =
R"""(Builds a VoxelOccupancyMap using the parameters specified.
VoxelOccupancyMaps are a dense voxel grid where each cell is a
voxelized_geometry_tools∷OccupancyCell that stores P(occupancy) as a
float. OccupancyMaps support computation of signed distance fields and
topological invariants ( of components, ============== of holes,

of voids). Empty cells receive occupancy=0.0, filled cells receive
occupancy=1.0.

Parameter ``plant``:
    MultibodyPlant model with a registered SceneGraph.

Parameter ``plant_context``:
    Context of plant.

Parameter ``geometries_to_ignore``:
    Set of geometries to ignore.

Parameter ``parent_body_name``:
    Name of parent body in the MultibodyPlant, used as frame name in
    the constructed OccupancyMap. If this name is not unique, or does
    not correspond to an existing MbP body, use
    override_parent_body_index to specfiy the parent body directly.

Parameter ``X_PG``:
    Pose of occupancy map frame G in frame of parent body P.

Parameter ``grid_dimensions``:
    Size of occupancy map in meters. If you specify a grid_size that
    is not evenly divisible by grid_resolution, you will get a larger
    grid with num_cells = ceil(size/resolution).

Parameter ``grid_resolution``:
    Cell size (in meters) for all Voxel grids used by the builder. All
    grids must have uniform cell sizes.

Precondition:
    grid_resolution > 0

Parameter ``override_parent_body_index``:
    Optionally provide a body index to override using parent_body_name
    to identify the parent body. Use this if the parent body name is
    not unique, or if the frame name does not match an existing MbP
    body (e.g. the name is a TF-compatible name incompatible with
    GetBodyByName).)""";
        } BuildOccupancyMap;
        // Symbol: drake::planning::experimental::BuildTaggedObjectOccupancyMap
        struct /* BuildTaggedObjectOccupancyMap */ {
          // Source: drake/planning/experimental/voxelized_environment_builder.h
          const char* doc =
R"""(Builds a VoxelTaggedObjectOccupancyMap using the parameters specified.
VoxelTaggedObjectOccupancyMaps are a dense voxel grid where each cell
is a voxelized_geometry_tools∷TaggedObjectOccupancyCell that stores
P(occupancy) as a float and object_id as a uint32_t.
TaggedObjectOccupancyMaps support computation of signed distance
fields and the first three topological invariants ( of components,
============== of holes,

of voids) as well as a limited form of spatial partioning. Empty cells
receive occupancy=0.0, filled cells receive occupancy=1.0. Filled
cells of the environment receive object_id values corresponding to the
integer values of the GeometryId in that location. If your environment
has more than 2^32 GeometryIds, this will throw.

Parameter ``plant``:
    MultibodyPlant model with a registered SceneGraph.

Parameter ``plant_context``:
    Context of plant.

Parameter ``geometries_to_ignore``:
    Set of geometries to ignore.

Parameter ``parent_body_name``:
    Name of parent body in the MultibodyPlant, used as frame name in
    the constructed OccupancyMap. If this name is not unique, or does
    not correspond to an existing MbP body, use
    override_parent_body_index to specfiy the parent body directly.

Parameter ``X_PG``:
    Pose of occupancy map frame G in frame of parent body P.

Parameter ``grid_dimensions``:
    Size of occupancy map in meters. If you specify a grid_size that
    is not evenly divisible by grid_resolution, you will get a larger
    grid with num_cells = ceil(size/resolution).

Parameter ``grid_resolution``:
    Cell size (in meters) for all Voxel grids used by the builder. All
    grids must have uniform cell sizes.

Precondition:
    grid_resolution > 0

Parameter ``override_parent_body_index``:
    Optionally provide a body index to override using parent_body_name
    to identify the parent body. Use this if the parent body name is
    not unique, or if the frame name does not match an existing MbP
    body (e.g. the name is a TF-compatible name incompatible with
    GetBodyByName).)""";
        } BuildTaggedObjectOccupancyMap;
        // Symbol: drake::planning::experimental::DistanceAndGradient
        struct /* DistanceAndGradient */ {
          // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
          const char* doc =
R"""(Wrapper that combines a distance value and gradient. This code uses
Vector4d because Vector4d allows for SIMD vector operations, which
Vector3d does not.)""";
          // Symbol: drake::planning::experimental::DistanceAndGradient::CollidingBodyIndex
          struct /* CollidingBodyIndex */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } CollidingBodyIndex;
          // Symbol: drake::planning::experimental::DistanceAndGradient::Distance
          struct /* Distance */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } Distance;
          // Symbol: drake::planning::experimental::DistanceAndGradient::DistanceAndGradient
          struct /* ctor */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::planning::experimental::DistanceAndGradient::Gradient
          struct /* Gradient */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } Gradient;
          // Symbol: drake::planning::experimental::DistanceAndGradient::Print
          struct /* Print */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } Print;
        } DistanceAndGradient;
        // Symbol: drake::planning::experimental::FillOccupancyMap
        struct /* FillOccupancyMap */ {
          // Source: drake/planning/experimental/voxelized_environment_builder.h
          const char* doc =
R"""(Fills an initialized VoxelOccupancyMap. Empty cells receive
occupancy=0.0, filled cells receive occupancy=1.0.

Parameter ``plant``:
    MultibodyPlant model with a registered SceneGraph.

Parameter ``plant_context``:
    Context of plant.

Parameter ``geometries_to_ignore``:
    Set of geometries to ignore.

Parameter ``occupancy_map``:
    TaggedObjectOccupancyMapGrid to fill.

Parameter ``override_parent_body_index``:
    Optionally provide a body index to override using the collision
    map frame name to identify the parent body. Use this if the frame
    name is not unique, or if the frame name does not match an
    existing MbP body (e.g. the name is a TF-compatible name
    incompatible with GetBodyByName).)""";
        } FillOccupancyMap;
        // Symbol: drake::planning::experimental::FillTaggedObjectOccupancyMap
        struct /* FillTaggedObjectOccupancyMap */ {
          // Source: drake/planning/experimental/voxelized_environment_builder.h
          const char* doc =
R"""(Fills an initialized VoxelTaggedObjectOccupancyMap. Empty cells
receive occupancy=0.0, filled cells receive occupancy=1.0. Filled
cells of the environment receive object_id values corresponding to the
integer values of the GeometryId in that location. If your environment
has more than 2^32 GeometryIds, this will throw.

Parameter ``plant``:
    MultibodyPlant model with a registered SceneGraph.

Parameter ``plant_context``:
    Context of plant.

Parameter ``geometries_to_ignore``:
    Set of geometries to ignore.

Parameter ``occupancy_map``:
    TaggedObjectOccupancyMapGrid to fill.

Parameter ``override_parent_body_index``:
    Optionally provide a body index to override using the collision
    map frame name to identify the parent body. Use this if the frame
    name is not unique, or if the frame name does not match an
    existing MbP body (e.g. the name is a TF-compatible name
    incompatible with GetBodyByName).)""";
        } FillTaggedObjectOccupancyMap;
        // Symbol: drake::planning::experimental::MbpEnvironmentCollisionChecker
        struct /* MbpEnvironmentCollisionChecker */ {
          // Source: drake/planning/experimental/mbp_environment_collision_checker.h
          const char* doc =
R"""(Sphere-model robot collision checker using MbP/SG to model environment
geometry.)""";
          // Symbol: drake::planning::experimental::MbpEnvironmentCollisionChecker::ComputePointToEnvironmentSignedDistanceAndGradient
          struct /* ComputePointToEnvironmentSignedDistanceAndGradient */ {
            // Source: drake/planning/experimental/mbp_environment_collision_checker.h
            const char* doc =
R"""(Query the (distance, gradient) of the provided point from obstacles.

Parameter ``context``:
    Context of the MbP model.

Parameter ``query_object``:
    Query object for ``context``.

Parameter ``p_WQ``:
    Query position in world frame W.

Parameter ``query_radius``:
    Gradients do not need to be computed for queries with distance >
    query_radius. This parameter is needed because the default
    implementation calls ComputePointSignedDistanceAndGradient, and
    only needing to check within a bound can improve performance.

Parameter ``X_WB_set``:
    Poses X_WB for all bodies in the model. Unused.

Parameter ``X_WB_inverse_set``:
    Poses X_BW for all bodies in the model. Unused.

Returns:
    pair<signed distance, gradient> where signed distance is positive
    if ``p_WQ`` is outside of objects, and negative if it is inside.
    The gradient is ∂d/∂p.)""";
          } ComputePointToEnvironmentSignedDistanceAndGradient;
          // Symbol: drake::planning::experimental::MbpEnvironmentCollisionChecker::MbpEnvironmentCollisionChecker
          struct /* ctor */ {
            // Source: drake/planning/experimental/mbp_environment_collision_checker.h
            const char* doc =
R"""(Creates a new checker with the given params.)""";
            // Source: drake/planning/experimental/mbp_environment_collision_checker.h
            const char* doc_copy =
R"""(To support Clone(), allow copying (but not move nor assign).)""";
          } ctor;
        } MbpEnvironmentCollisionChecker;
        // Symbol: drake::planning::experimental::Placeholder
        struct /* Placeholder */ {
          // Source: drake/planning/experimental/placeholder.h
          const char* doc =
R"""(This is a stub class to exercise the build system.)""";
          // Symbol: drake::planning::experimental::Placeholder::Placeholder
          struct /* ctor */ {
            // Source: drake/planning/experimental/placeholder.h
            const char* doc =
R"""(This is a stub constructor to exercise the build system.)""";
          } ctor;
        } Placeholder;
        // Symbol: drake::planning::experimental::PointSignedDistanceAndGradientResult
        struct /* PointSignedDistanceAndGradientResult */ {
          // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
          const char* doc =
R"""(Wrapper for multiple distance and gradient values. Keeps track of the
minimum distance of all DistanceAndGradients added.)""";
          // Symbol: drake::planning::experimental::PointSignedDistanceAndGradientResult::AddDistance
          struct /* AddDistance */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } AddDistance;
          // Symbol: drake::planning::experimental::PointSignedDistanceAndGradientResult::AddDistanceAndGradient
          struct /* AddDistanceAndGradient */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } AddDistanceAndGradient;
          // Symbol: drake::planning::experimental::PointSignedDistanceAndGradientResult::GetDistanceAndGradient
          struct /* GetDistanceAndGradient */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } GetDistanceAndGradient;
          // Symbol: drake::planning::experimental::PointSignedDistanceAndGradientResult::MinimumCollidingBodyIndex
          struct /* MinimumCollidingBodyIndex */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } MinimumCollidingBodyIndex;
          // Symbol: drake::planning::experimental::PointSignedDistanceAndGradientResult::MinimumDistance
          struct /* MinimumDistance */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } MinimumDistance;
          // Symbol: drake::planning::experimental::PointSignedDistanceAndGradientResult::NumberOfGradients
          struct /* NumberOfGradients */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } NumberOfGradients;
          // Symbol: drake::planning::experimental::PointSignedDistanceAndGradientResult::PointSignedDistanceAndGradientResult
          struct /* ctor */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::planning::experimental::PointSignedDistanceAndGradientResult::Print
          struct /* Print */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } Print;
        } PointSignedDistanceAndGradientResult;
        // Symbol: drake::planning::experimental::SelfFilter
        struct /* SelfFilter */ {
          // Source: drake/planning/experimental/voxel_self_filter.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Self-filter implementation for VoxelOccupancyMap environments.
Self-filter marks voxels belonging to the robot as empty so that they
do not produce false collisions in a voxelized environment used for
collision checking.

Parameter ``collision_checker``:
    Sphere-model collision checker that provides the sphere model of
    robot geometry and performs forward kinematics.

Parameter ``q``:
    Current configuration of the robot.

Parameter ``padding``:
    Padding to inflate the spheres of the collision model to use in
    the self-filter.

Precondition:
    >= 0.0.

Parameter ``occupancy_map``:
    Current environment.

Precondition:
    != nullptr.

Parameter ``parallelism``:
    Parallelism to use.

Parameter ``context_number``:
    Optional context number for use in parallel contexts.)""";
        } SelfFilter;
        // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker
        struct /* SphereRobotModelCollisionChecker */ {
          // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
          const char* doc =
R"""(Base class for collision checkers using a sphere-geometry robot model.
Derived classes must implement distance and gradient checks against
environment geometry. Note: this class is designed such that derived
classes can support copy/move/assignment.)""";
          // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker::AddEnvironmentCollisionShapeToBody
          struct /* AddEnvironmentCollisionShapeToBody */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } AddEnvironmentCollisionShapeToBody;
          // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker::CheckSelfCollisionFree
          struct /* CheckSelfCollisionFree */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc =
R"""(Check if the robot is in self-collision.

Parameter ``X_WB_set``:
    Vector of X_WB, pose of robot body B in world W.

Returns:
    true if self-collision free, false otherwise.)""";
          } CheckSelfCollisionFree;
          // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker::ComputeBodyPoses
          struct /* ComputeBodyPoses */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc =
R"""(Compute pose of all bodies, using the current thread's context.

Parameter ``q``:
    Configuration to compute poses for.

Returns:
    Vector of X_WB, pose of body B in world W, for all bodies.)""";
          } ComputeBodyPoses;
          // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker::ComputePointToEnvironmentSignedDistance
          struct /* ComputePointToEnvironmentSignedDistance */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc =
R"""(Query the distance of the provided point from obstacles.

Parameter ``context``:
    Context of the MbP model.

Parameter ``query_object``:
    Query object for ``context``.

Parameter ``p_WQ``:
    Query position in world frame W.

Parameter ``query_radius``:
    Gradients do not need to be computed for queries with distance >
    query_radius. This parameter is needed because the default
    implementation calls ComputePointSignedDistanceAndGradient, and
    only needing to check within a bound can improve performance.

Parameter ``X_WB_set``:
    Poses X_WB for all bodies in the model. This is used to move
    gradients from signed distance fields back to world frame.

Parameter ``X_WB_inverse_set``:
    Poses X_BW for all bodies in the model. This is useful for some
    implementations to move the provided p_WQ into a different body
    frame.

Returns:
    signed distances where signed distance is positive if ``p_WQ`` is
    outside of objects, and negative if it is inside. The default
    implementation here is always sufficient, but in some cases, it
    may be faster for an implementation to only compute the distance,
    rather than the distance gradient. Since the binary collision
    checking methods only need distance, we want them to be as fast as
    possible. Note that queries farther than ``query_radius`` from
    collision need not return exact distance.)""";
          } ComputePointToEnvironmentSignedDistance;
          // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker::ComputePointToEnvironmentSignedDistanceAndGradient
          struct /* ComputePointToEnvironmentSignedDistanceAndGradient */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc =
R"""(Query the (distance, gradient) of the provided point from obstacles.

Parameter ``context``:
    Context of the MbP model.

Parameter ``query_object``:
    Query object for ``context``.

Parameter ``p_WQ``:
    Query position in world frame W.

Parameter ``query_radius``:
    Gradients do not need to be computed for queries with distance >
    query radius. This allows for faster performance by reducing the
    number of gradients to compute as well as reducing the number of
    gradients this method returns.

Parameter ``X_WB_set``:
    Poses X_WB for all bodies in the model. This is used to move
    gradients from signed distance fields back to world frame.

Parameter ``X_WB_inverse_set``:
    Poses X_BW for all bodies in the model. This is useful for some
    implementations to move the provided p_WQ into a different body
    frame.

Returns:
    signed distances and gradients where signed distance is positive
    if ``p_WQ`` is outside of objects, and negative if it is inside.
    The gradient is ∂d/∂p. Note that queries farther than
    ``query_radius`` from collision need not return exact distance.)""";
          } ComputePointToEnvironmentSignedDistanceAndGradient;
          // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker::ComputeSelfCollisionSignedDistanceAndGradient
          struct /* ComputeSelfCollisionSignedDistanceAndGradient */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } ComputeSelfCollisionSignedDistanceAndGradient;
          // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker::ComputeSphereLocationsInWorldFrame
          struct /* ComputeSphereLocationsInWorldFrame */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc_1args =
R"""(Compute sphere collision model in world frame.

Parameter ``X_WB_set``:
    Body poses to use.

Returns:
    Vector of link sphere models.)""";
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc_2args =
R"""(Compute sphere collision model in world frame, using the current
thread's context.

Parameter ``q``:
    Configuration to compute poses for.

Returns:
    Vector of link sphere models.)""";
          } ComputeSphereLocationsInWorldFrame;
          // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker::EstimateConservativePointToEnvironmentSignedDistance
          struct /* EstimateConservativePointToEnvironmentSignedDistance */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc =
R"""(Query a conservative underestimate of the distance of the provided
point from obstacles.

Parameter ``context``:
    Context of the MbP model.

Parameter ``query_object``:
    Query object for ``context``.

Parameter ``p_WQ``:
    Query position in world frame W.

Parameter ``query_radius``:
    If no obstacles are within this distance of ``p_WQ``, infinity (or
    any value greater than ``query_radius``) may be returned to avoid
    the work of computing a more exact distance.

Parameter ``X_WB_set``:
    Poses X_WB for all bodies in the model. This is used to move
    gradients from signed distance fields back to world frame.

Parameter ``X_WB_inverse_set``:
    Poses X_BW for all bodies in the model. This is useful for some
    implementations to move the provided p_WQ into a different body
    frame.

Returns:
    a conservative underestimate of the distance between the provided
    point ``p_WQ`` and obstacles. If no such estimate is possible,
    nullopt must be returned.)""";
          } EstimateConservativePointToEnvironmentSignedDistance;
          // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker::GetBodyPoses
          struct /* GetBodyPoses */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc =
R"""(Get pose of all bodies.

Parameter ``plant_context``:
    MbP context, already updated, to use.

Returns:
    Vector of X_WB, pose of body B in world W, for all bodies.)""";
          } GetBodyPoses;
          // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker::GetURDFCollisionGeometriesForRobotCollisionModel
          struct /* GetURDFCollisionGeometriesForRobotCollisionModel */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc =
R"""(Generates the URDF <collision> elements corresponding to the robot's
collision model.)""";
          } GetURDFCollisionGeometriesForRobotCollisionModel;
          // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker::RemoveAllAddedEnvironment
          struct /* RemoveAllAddedEnvironment */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } RemoveAllAddedEnvironment;
          // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker::RobotCollisionModel
          struct /* RobotCollisionModel */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc =
R"""(Get a reference to the internal collision model of the robot.)""";
          } RobotCollisionModel;
          // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker::RobotGeometries
          struct /* RobotGeometries */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } RobotGeometries;
          // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker::SphereRobotModelCollisionChecker
          struct /* ctor */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc =
R"""(Construct a base collision checker for sphere-geometry robot models.)""";
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc_copy =
R"""(To support Clone(), allow copying (but not move nor assign).)""";
          } ctor;
          // Symbol: drake::planning::experimental::SphereRobotModelCollisionChecker::UpdateBodyCollisionModel
          struct /* UpdateBodyCollisionModel */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc =
R"""(Update the sphere collision model of a specific body of the robot.

Parameter ``body_index``:
    Index of the body (body 0 is world).

Parameter ``spheres``:
    Sphere geometry to add to the collision model.

Parameter ``append``:
    If true, appends the spheres to the existing model, otherwise the
    new model replaces the existing model. Note: changes to the
    collision model here *do not* go through the added geometry
    mechanism.)""";
          } UpdateBodyCollisionModel;
        } SphereRobotModelCollisionChecker;
        // Symbol: drake::planning::experimental::SphereSpecification
        struct /* SphereSpecification */ {
          // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
          const char* doc =
R"""(Class modelling collision spheres used for collision checking. This
code uses Vector4d because Vector4d allows for SIMD vector operations
when performing Isometry3d * Vector4d, which Vector3d does not.)""";
          // Symbol: drake::planning::experimental::SphereSpecification::Origin
          struct /* Origin */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } Origin;
          // Symbol: drake::planning::experimental::SphereSpecification::Radius
          struct /* Radius */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } Radius;
          // Symbol: drake::planning::experimental::SphereSpecification::SetOrigin
          struct /* SetOrigin */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc = R"""()""";
          } SetOrigin;
          // Symbol: drake::planning::experimental::SphereSpecification::SphereSpecification
          struct /* ctor */ {
            // Source: drake/planning/experimental/sphere_robot_model_collision_checker.h
            const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Make a sphere.

Parameter ``p_BSo``:
    Origin of sphere S in frame of body B.

Parameter ``radius``:
    Radius of sphere.)""";
          } ctor;
        } SphereSpecification;
        // Symbol: drake::planning::experimental::VoxelOccupancyMap
        struct /* VoxelOccupancyMap */ {
          // Source: drake/planning/experimental/voxel_occupancy_map.h
          const char* doc = R"""()""";
          // Symbol: drake::planning::experimental::VoxelOccupancyMap::ExportSignedDistanceField
          struct /* ExportSignedDistanceField */ {
            // Source: drake/planning/experimental/voxel_occupancy_map.h
            const char* doc =
R"""(Construct a voxelized signed distance field from ``this`` using the
provided parameters ``parameters``.)""";
          } ExportSignedDistanceField;
          // Symbol: drake::planning::experimental::VoxelOccupancyMap::VoxelOccupancyMap
          struct /* ctor */ {
            // Source: drake/planning/experimental/voxel_occupancy_map.h
            const char* doc_0args =
R"""(Default constructor creates an empty VoxelOccupancyMap.)""";
            // Source: drake/planning/experimental/voxel_occupancy_map.h
            const char* doc_5args_parent_body_name_X_PG_grid_dimensions_grid_resolution_default_occupancy =
R"""(Construct a VoxelOccupancyMap filled with cells of
``default_occupancy`` and ``default_object_id``. `parent_body_name`
specifies the name of the parent body, and ``X_PG`` specifies the pose
of the origin of the voxel grid relative to the parent body frame.
``grid_dimensions`` specifies the nominal dimensions of the voxel
grid, and ``grid_resolution`` specifies the size of an individual
voxel. If you specify dimensions that are not evenly divisible by
``grid_resolution``, you will get a larger grid with num_cells =
ceil(dimension/resolution).)""";
            // Source: drake/planning/experimental/voxel_occupancy_map.h
            const char* doc_5args_parent_body_name_X_PG_grid_counts_grid_resolution_default_occupancy =
R"""(Construct a VoxelOccupancyMap filled with cells of
``default_occupancy`` and ``default_object_id``. `parent_body_name`
specifies the name of the parent body, and ``X_PG`` specifies the pose
of the origin of the voxel grid relative to the parent body frame.
``grid_counts`` specifies the number of voxels for each axis of the
voxel grid, and ``grid_resolution`` specifies the size of an
individual voxel.)""";
            // Source: drake/planning/experimental/voxel_occupancy_map.h
            const char* doc_copy =
R"""(VoxelOccupancyMap provides copy, move, and assignment.)""";
          } ctor;
          // Symbol: drake::planning::experimental::VoxelOccupancyMap::internal_representation
          struct /* internal_representation */ {
            // Source: drake/planning/experimental/voxel_occupancy_map.h
            const char* doc = R"""()""";
          } internal_representation;
          // Symbol: drake::planning::experimental::VoxelOccupancyMap::is_empty
          struct /* is_empty */ {
            // Source: drake/planning/experimental/voxel_occupancy_map.h
            const char* doc =
R"""(Returns true if empty. A VoxelOccupancyMap will be empty if it is in
the default constructed state or has been moved-from.)""";
          } is_empty;
          // Symbol: drake::planning::experimental::VoxelOccupancyMap::mutable_internal_representation
          struct /* mutable_internal_representation */ {
            // Source: drake/planning/experimental/voxel_occupancy_map.h
            const char* doc = R"""()""";
          } mutable_internal_representation;
          // Symbol: drake::planning::experimental::VoxelOccupancyMap::parent_body_name
          struct /* parent_body_name */ {
            // Source: drake/planning/experimental/voxel_occupancy_map.h
            const char* doc = R"""(Get the name of the parent body frame.)""";
          } parent_body_name;
        } VoxelOccupancyMap;
        // Symbol: drake::planning::experimental::VoxelSignedDistanceField
        struct /* VoxelSignedDistanceField */ {
          // Source: drake/planning/experimental/voxel_signed_distance_field.h
          const char* doc =
R"""(Container for voxelized signed distance fields. To enable efficient
sharing signed distance fields (which may be quite large) between
multiple uses, a VoxelSignedDistanceField operates equivalently to
shared_ptr<const T> for the underlying voxelized signed distance
field.)""";
          // Symbol: drake::planning::experimental::VoxelSignedDistanceField::GenerationParameters
          struct /* GenerationParameters */ {
            // Source: drake/planning/experimental/voxel_signed_distance_field.h
            const char* doc =
R"""(Param struct for generating a VoxelSignedDistanceField.)""";
            // Symbol: drake::planning::experimental::VoxelSignedDistanceField::GenerationParameters::add_virtual_border
            struct /* add_virtual_border */ {
              // Source: drake/planning/experimental/voxel_signed_distance_field.h
              const char* doc =
R"""(Should a "virtual border" be added to the occpancy grid, such that all
cells at the edges of the occupancy grid are assumed to be on the
border of filled/empty space? This should only be enabled for specific
uses, such as certain grasp search problems.)""";
            } add_virtual_border;
            // Symbol: drake::planning::experimental::VoxelSignedDistanceField::GenerationParameters::oob_value
            struct /* oob_value */ {
              // Source: drake/planning/experimental/voxel_signed_distance_field.h
              const char* doc =
R"""(Out-of-bounds value stored in the VoxelSignedDistanceField.)""";
            } oob_value;
            // Symbol: drake::planning::experimental::VoxelSignedDistanceField::GenerationParameters::parallelism
            struct /* parallelism */ {
              // Source: drake/planning/experimental/voxel_signed_distance_field.h
              const char* doc =
R"""(Parallelism to use when generating the VoxelSignedDistanceField.)""";
            } parallelism;
            // Symbol: drake::planning::experimental::VoxelSignedDistanceField::GenerationParameters::unknown_is_filled
            struct /* unknown_is_filled */ {
              // Source: drake/planning/experimental/voxel_signed_distance_field.h
              const char* doc =
R"""(If cells with unknown occupancy (i.e. 0.5) should be treated as filled
(i.e. occupancy > 0.5) or empty (i.e. occupancy < 0.5) for the
purposes of generating the VoxelSignedDistanceField.)""";
            } unknown_is_filled;
          } GenerationParameters;
          // Symbol: drake::planning::experimental::VoxelSignedDistanceField::VoxelSignedDistanceField
          struct /* ctor */ {
            // Source: drake/planning/experimental/voxel_signed_distance_field.h
            const char* doc =
R"""(Default constructor creates an empty VoxelSignedDistanceField.)""";
            // Source: drake/planning/experimental/voxel_signed_distance_field.h
            const char* doc_copy =
R"""(VoxelSignedDistanceField provides copy, move, and assignment.)""";
          } ctor;
          // Symbol: drake::planning::experimental::VoxelSignedDistanceField::internal_representation
          struct /* internal_representation */ {
            // Source: drake/planning/experimental/voxel_signed_distance_field.h
            const char* doc = R"""()""";
          } internal_representation;
          // Symbol: drake::planning::experimental::VoxelSignedDistanceField::is_empty
          struct /* is_empty */ {
            // Source: drake/planning/experimental/voxel_signed_distance_field.h
            const char* doc =
R"""(Returns true if empty. A VoxelSignedDistanceField will be empty if it
is in the default constructed state or has been moved-from.)""";
          } is_empty;
          // Symbol: drake::planning::experimental::VoxelSignedDistanceField::parent_body_name
          struct /* parent_body_name */ {
            // Source: drake/planning/experimental/voxel_signed_distance_field.h
            const char* doc = R"""(Get the name of the parent body frame.)""";
          } parent_body_name;
        } VoxelSignedDistanceField;
        // Symbol: drake::planning::experimental::VoxelTaggedObjectOccupancyMap
        struct /* VoxelTaggedObjectOccupancyMap */ {
          // Source: drake/planning/experimental/voxel_tagged_object_occupancy_map.h
          const char* doc = R"""()""";
          // Symbol: drake::planning::experimental::VoxelTaggedObjectOccupancyMap::ExportSignedDistanceField
          struct /* ExportSignedDistanceField */ {
            // Source: drake/planning/experimental/voxel_tagged_object_occupancy_map.h
            const char* doc =
R"""(Construct a voxelized signed distance field from ``this`` using the
provided parameters ``parameters``. The objects to include are
specified by ``objects_to_include``; if left empty all objects will be
included.)""";
          } ExportSignedDistanceField;
          // Symbol: drake::planning::experimental::VoxelTaggedObjectOccupancyMap::VoxelTaggedObjectOccupancyMap
          struct /* ctor */ {
            // Source: drake/planning/experimental/voxel_tagged_object_occupancy_map.h
            const char* doc_0args =
R"""(Default constructor creates an empty VoxelOccupancyMap.)""";
            // Source: drake/planning/experimental/voxel_tagged_object_occupancy_map.h
            const char* doc_6args_parent_body_name_X_PG_grid_dimensions_grid_resolution_default_occupancy_default_object_id =
R"""(Construct a VoxelTaggedObjectOccupancyMap filled with cells of
``default_occupancy`` and ``default_object_id``. `parent_body_name`
specifies the name of the parent body, and ``X_PG`` specifies the pose
of the origin of the voxel grid relative to the parent body frame.
``grid_dimensions`` specifies the nominal dimensions of the voxel
grid, and ``grid_resolution`` specifies the size of an individual
voxel. If you specify dimensions that are not evenly divisible by
``grid_resolution``, you will get a larger grid with num_cells =
ceil(dimension/resolution).)""";
            // Source: drake/planning/experimental/voxel_tagged_object_occupancy_map.h
            const char* doc_6args_parent_body_name_X_PG_grid_counts_grid_resolution_default_occupancy_default_object_id =
R"""(Construct a VoxelTaggedObjectOccupancyMap filled with cells of
``default_occupancy`` and ``default_object_id``. `parent_body_name`
specifies the name of the parent body, and ``X_PG`` specifies the pose
of the origin of the voxel grid relative to the parent body frame.
``grid_counts`` specifies the number of voxels for each axis of the
voxel grid, and ``grid_resolution`` specifies the size of an
individual voxel.)""";
            // Source: drake/planning/experimental/voxel_tagged_object_occupancy_map.h
            const char* doc_copy =
R"""(VoxelTaggedObjectOccupancyMap provides copy, move, and assignment.)""";
          } ctor;
          // Symbol: drake::planning::experimental::VoxelTaggedObjectOccupancyMap::internal_representation
          struct /* internal_representation */ {
            // Source: drake/planning/experimental/voxel_tagged_object_occupancy_map.h
            const char* doc = R"""()""";
          } internal_representation;
          // Symbol: drake::planning::experimental::VoxelTaggedObjectOccupancyMap::is_empty
          struct /* is_empty */ {
            // Source: drake/planning/experimental/voxel_tagged_object_occupancy_map.h
            const char* doc =
R"""(Returns true if empty. A VoxelTaggedObjectOccupancyMap will be empty
if it is in the default constructed state or has been moved-from.)""";
          } is_empty;
          // Symbol: drake::planning::experimental::VoxelTaggedObjectOccupancyMap::mutable_internal_representation
          struct /* mutable_internal_representation */ {
            // Source: drake/planning/experimental/voxel_tagged_object_occupancy_map.h
            const char* doc = R"""()""";
          } mutable_internal_representation;
          // Symbol: drake::planning::experimental::VoxelTaggedObjectOccupancyMap::parent_body_name
          struct /* parent_body_name */ {
            // Source: drake/planning/experimental/voxel_tagged_object_occupancy_map.h
            const char* doc = R"""(Get the name of the parent body frame.)""";
          } parent_body_name;
        } VoxelTaggedObjectOccupancyMap;
        // Symbol: drake::planning::experimental::VoxelizedEnvironmentCollisionChecker
        struct /* VoxelizedEnvironmentCollisionChecker */ {
          // Source: drake/planning/experimental/voxelized_environment_collision_checker.h
          const char* doc =
R"""(Collision checker using a voxelized environment model.)""";
          // Symbol: drake::planning::experimental::VoxelizedEnvironmentCollisionChecker::ComputePointToEnvironmentSignedDistance
          struct /* ComputePointToEnvironmentSignedDistance */ {
            // Source: drake/planning/experimental/voxelized_environment_collision_checker.h
            const char* doc =
R"""(Query the distance of the provided point from obstacles.

Parameter ``plant_context``:
    Context of the MbP model. Unused.

Parameter ``query_object``:
    Query object retrieved from ``plant_context``.

Parameter ``p_WQ``:
    Query position in world frame W.

Parameter ``query_radius``:
    Gradients do not need to be computed for queries with distance >
    query_radius. This parameter is needed because the default
    implementation calls ComputePointSignedDistanceAndGradient, and
    only needing to check within a bound can improve performance.

Parameter ``X_WB_set``:
    Poses X_WB for all bodies in the model. This is used to move
    gradients from signed distance fields back to world frame.

Parameter ``X_WB_inverse_set``:
    Poses X_BW for all bodies in the model. This is used to move the
    provided p_WQ into the frame of signed distance fields.

Returns:
    signed distances where signed distance is positive if ``p_WQ`` is
    outside of objects, and negative if it is inside.)""";
          } ComputePointToEnvironmentSignedDistance;
          // Symbol: drake::planning::experimental::VoxelizedEnvironmentCollisionChecker::ComputePointToEnvironmentSignedDistanceAndGradient
          struct /* ComputePointToEnvironmentSignedDistanceAndGradient */ {
            // Source: drake/planning/experimental/voxelized_environment_collision_checker.h
            const char* doc =
R"""(Query the (distance, gradient) of the provided point from obstacles.

Parameter ``plant_context``:
    Context of the MbP model. Unused.

Parameter ``query_object``:
    Query object retrieved from ``plant_context``.

Parameter ``p_WQ``:
    Query position in world frame W.

Parameter ``query_radius``:
    Gradients do not need to be computed for queries with distance >
    query_radius. This parameter is needed because the default
    implementation calls ComputePointSignedDistanceAndGradient, and
    only needing to check within a bound can improve performance.

Parameter ``X_WB_set``:
    Poses X_WB for all bodies in the model. This is used to move
    gradients from signed distance fields back to world frame.

Parameter ``X_WB_inverse_set``:
    Poses X_BW for all bodies in the model. This is used to move the
    provided p_WQ into the frame of signed distance fields.

Returns:
    signed distances and gradients, where signed distance is positive
    if ``p_WQ`` is outside of objects, and negative if it is inside.
    The gradient is ∂d/∂p.)""";
          } ComputePointToEnvironmentSignedDistanceAndGradient;
          // Symbol: drake::planning::experimental::VoxelizedEnvironmentCollisionChecker::EnvironmentSDFBodies
          struct /* EnvironmentSDFBodies */ {
            // Source: drake/planning/experimental/voxelized_environment_collision_checker.h
            const char* doc = R"""()""";
          } EnvironmentSDFBodies;
          // Symbol: drake::planning::experimental::VoxelizedEnvironmentCollisionChecker::EnvironmentSDFs
          struct /* EnvironmentSDFs */ {
            // Source: drake/planning/experimental/voxelized_environment_collision_checker.h
            const char* doc = R"""()""";
          } EnvironmentSDFs;
          // Symbol: drake::planning::experimental::VoxelizedEnvironmentCollisionChecker::RemoveEnvironment
          struct /* RemoveEnvironment */ {
            // Source: drake/planning/experimental/voxelized_environment_collision_checker.h
            const char* doc =
R"""(Remove the voxelized model corresponding to ``environment_name``.)""";
          } RemoveEnvironment;
          // Symbol: drake::planning::experimental::VoxelizedEnvironmentCollisionChecker::UpdateEnvironment
          struct /* UpdateEnvironment */ {
            // Source: drake/planning/experimental/voxelized_environment_collision_checker.h
            const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Update the voxelized environment.

Parameter ``environment_name``:
    Name of the environment model to update. If the name is already in
    use, the new model replaces the old. To remove a model, provide a
    default-constructed VoxelOccupancyMap that is not initialized.

Parameter ``environment``:
    Voxelized environment model.

Parameter ``override_environment_body_index``:
    Optionally provide a body index to override the environment frame
    name -> body lookup. Use this if the frame name is not unique, or
    if the frame name does not match an existing MbP body.)""";
          } UpdateEnvironment;
          // Symbol: drake::planning::experimental::VoxelizedEnvironmentCollisionChecker::VoxelizedEnvironmentCollisionChecker
          struct /* ctor */ {
            // Source: drake/planning/experimental/voxelized_environment_collision_checker.h
            const char* doc =
R"""(Creates a new checker with the given params.)""";
            // Source: drake/planning/experimental/voxelized_environment_collision_checker.h
            const char* doc_copy =
R"""(To support Clone(), allow copying (but not move nor assign).)""";
          } ctor;
        } VoxelizedEnvironmentCollisionChecker;
      } experimental;
    } planning;
  } drake;
} pydrake_doc_planning_experimental;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif

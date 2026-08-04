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

// #include "drake/multibody/benchmarks/inclined_plane/inclined_plane_plant.h"

// Symbol: pydrake_doc_multibody_benchmarks_inclined_plane
constexpr struct /* pydrake_doc_multibody_benchmarks_inclined_plane */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::benchmarks
      struct /* benchmarks */ {
        // Symbol: drake::multibody::benchmarks::inclined_plane
        struct /* inclined_plane */ {
          // Symbol: drake::multibody::benchmarks::inclined_plane::AddInclinedPlaneAndGravityToPlant
          struct /* AddInclinedPlaneAndGravityToPlant */ {
            // Source: drake/multibody/benchmarks/inclined_plane/inclined_plane_plant.h
            const char* doc =
R"""(Creates an inclined plane A and adds it to an existing plant.

See also:
    inclined_plane_parameters "Description of parameters")""";
          } AddInclinedPlaneAndGravityToPlant;
          // Symbol: drake::multibody::benchmarks::inclined_plane::AddInclinedPlaneWithBlockToPlant
          struct /* AddInclinedPlaneWithBlockToPlant */ {
            // Source: drake/multibody/benchmarks/inclined_plane/inclined_plane_plant.h
            const char* doc =
R"""(Creates an inclined plane A and a uniform-density block (body B),
optionally with 4 spheres welded to it, and adds them to an existing
plant.

Parameter ``block_dimensions``:
    Dimensions (lengths) of block in the Bx, By, Bz directions
    (meters). To be valid data, these dimensions must be positive.

Parameter ``is_block_with_4Spheres``:
    This flag is ``True`` if block B's contact with inclined plane A
    is modeled using 4 identical massless spheres welded to the block
    B's four "bottom" corners, whereas this flag is `false`if block
    B's contact is modeled with a block (box).

See also:
    inclined_plane_parameters "Description of other parameters")""";
          } AddInclinedPlaneWithBlockToPlant;
          // Symbol: drake::multibody::benchmarks::inclined_plane::AddInclinedPlaneWithSphereToPlant
          struct /* AddInclinedPlaneWithSphereToPlant */ {
            // Source: drake/multibody/benchmarks/inclined_plane/inclined_plane_plant.h
            const char* doc =
R"""(Creates an inclined plane A and a uniform-density sphere (body B) and
adds them to an existing plant.

Parameter ``radiusB``:
    The radius of sphere B (meters), which must be positive.

See also:
    inclined_plane_parameters "Description of other parameters"

Note:
    Decorative visual geometry is added to the sphere to facilitate
    visualizing the sphere's rotation.)""";
          } AddInclinedPlaneWithSphereToPlant;
        } inclined_plane;
      } benchmarks;
    } multibody;
  } drake;
} pydrake_doc_multibody_benchmarks_inclined_plane;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif

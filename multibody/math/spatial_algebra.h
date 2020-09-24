#pragma once

/// @file
/// This is the entry point for all operations with spatial vectors.
/// Spatial vectors represent spatial physical quantities such as spatial
/// velocities, spatial accelerations and spatial forces. Spatial vectors are
/// 6-element quantities that are pairs of ordinary 3-vectors. Elements 0-2 are
/// always the rotational component while elements 3-5 are always the
/// translational component.
/// For a more detailed introduction on spatial vectors please refer to
/// section @ref multibody_spatial_vectors.

#define DRAKE_SPATIAL_ALGEBRA_HEADER
#include "drake/multibody/math/spatial_acceleration.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/math/spatial_momentum.h"
#include "drake/multibody/math/spatial_velocity.h"
#undef DRAKE_SPATIAL_ALGEBRA_HEADER

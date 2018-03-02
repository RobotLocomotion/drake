#pragma once

// TODO(mitiguy) Merge Rotation class from
// drake/multibody/multibody_tree/math/rotation.h into
// drake/math/rotation_matrix.h.
// The goal of PR #8217 is a first step to replace the "old" rotation_matrix.h
// file with the "new" rotation_matrix.h file which contains the new
// RotationMatrix class.  The process being employed will be able to make this
// replacement without changing #include drake/math/rotation_matrix.h statements
// in the multiplicity of its uses in currently existing Drake code.
// For PR #8217, the process is as follows:
// * Rename drake/math/rotation_matrix.h to
//   drake/math/rotation_matrix_utilities.h.
// * Rename drake/math/rotation_matrix.cc to
//   drake/math/rotation_matrix_utilities.cc.
// * Create new (mostly-empty) files:
//   drake/math/rotation_matrix.h
//   drake/math/rotation_matrix.cc.
// * Update build files and #include statements accordingly.
#include "drake/math/rotation_matrix_utilities.h"

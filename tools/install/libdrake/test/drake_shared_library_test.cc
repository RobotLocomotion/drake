// This test is meant to consume `drake_shared_libary` only to test for things
// such as ODR violations using ASan.

// Trivial include.
#include "drake/common/find_resource.h"

// Trivial symbol test.
using drake::FindResourceOrThrow;

// No-op.
int main() { return 0; }

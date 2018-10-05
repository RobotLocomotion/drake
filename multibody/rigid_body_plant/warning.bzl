# -*- python -*-

WARNING = """Deprecation message...
Several of Drake's Bazel packages are deprecated, and will be removed soon:
 //multibody/collision
 //multibody/joints
 //multibody/parsers
 //multibody/rigid_body_plant
 //multibody/shapes
Users should change their 'deps = []' or 'data = []' declarations to refer
to the new names of these package by prepending 'attic/' to the name, as in:
 //attic/multibody/collision
 //attic/multibody/joints
 //attic/multibody/parsers
 //attic/multibody/rigid_body_plant
 //attic/multibody/shapes
"""

# This warning will be printed at most once (at load-time), anytime someone
# mentions one of the deprecated packages.
print(WARNING)

# N.B. When this file is deleted (when the packages are deleted), remove the
# recently added --deleted_packages flags from drake/tools/bazel.rc.

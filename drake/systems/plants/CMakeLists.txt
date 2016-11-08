# TODO(amcastro-tri): Remove these once director is updated to use header files
# in drake/multibody/. See issue #4058.
drake_install_headers(
        ForceTorqueMeasurement.h
        RigidBodyTree.h)

add_subdirectory(constraint)
add_subdirectory(shapes)
add_subdirectory(spring_mass_system)
add_subdirectory(test)

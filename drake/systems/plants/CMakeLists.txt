drake_install_headers(
        ForceTorqueMeasurement.h
        RigidBodyTree.h)

add_subdirectory(constraint)
add_subdirectory(rigid_body_plant)
add_subdirectory(shapes)
add_subdirectory(spring_mass_system)
add_subdirectory(test)

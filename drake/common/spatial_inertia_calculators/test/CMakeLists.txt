# Rules for unit tests of spatial inertia calculators.
if(PYTHONINTERP_FOUND)
  add_test(NAME spatial_inertia_solid_cuboid_test COMMAND
    ${PYTHON_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/spatial_inertia_solid_cuboid_test.py)
endif()

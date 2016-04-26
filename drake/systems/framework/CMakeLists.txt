
add_library_with_exports(LIB_NAME drakeSystemFramework
                         SOURCE_FILES value.cc value.h)
pods_install_libraries(drakeSystemFramework)
drake_install_headers(value.h)
pods_install_pkg_config_file(drake-system-framework
    LIBS -ldrakeSystemFramework
    REQUIRES
    VERSION 0.0.1)

drake_install_headers(value.h)

if (GTEST_FOUND)
	add_executable(value_test value_test.cc)
	target_link_libraries(value_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
	add_test(NAME value_test COMMAND value_test)
endif()

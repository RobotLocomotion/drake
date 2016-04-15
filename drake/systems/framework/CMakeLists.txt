
add_library_with_exports(LIB_NAME drakeSystemFramework
                         SOURCE_FILES abstract_value.cc
                                      value.cc)

drake_install_headers(abstract_value.h value.h)

if (GTEST_FOUND)
	add_executable(value_test value_test.cc)
	target_link_libraries(value_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
	add_test(NAME value_test COMMAND value_test)
endif()
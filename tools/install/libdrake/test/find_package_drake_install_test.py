import os
import subprocess
import textwrap
import unittest

import install_test_helper


class FindPackageDrakeInstallTest(unittest.TestCase):
    def test_find_package_drake(self):
        cmake_source_dir = install_test_helper.create_temporary_dir("src")

        cc_content_drake = """
            #include <drake/common/symbolic/expression.h>
            int main() {
              drake::symbolic::Environment environment;
              return 0;
            }
        """

        cc_filename_drake = os.path.join(cmake_source_dir, "main_drake.cc")

        with open(cc_filename_drake, "w") as f:
            f.write(textwrap.dedent(cc_content_drake))

        cmake_prefix_path = install_test_helper.get_install_dir()

        cmake_content = """
            cmake_minimum_required(VERSION 3.9...4.2)
            project(find_package_drake_install_test)
            set(CMAKE_PREFIX_PATH {cmake_prefix_path})
            find_package(drake CONFIG REQUIRED)
            add_executable(main_drake main_drake.cc)
            target_link_libraries(main_drake drake::drake)

            # Check that the imported drake::drake is a shared library.
            get_target_property(drake_type drake::drake TYPE)
            if(NOT drake_type STREQUAL "SHARED_LIBRARY")
                message(FATAL_ERROR "drake::drake is ${{drake_type}}, but expected SHARED_LIBRARY.")
            endif()
        """.format(cmake_prefix_path=cmake_prefix_path)

        cmake_filename = os.path.join(cmake_source_dir, "CMakeLists.txt")

        with open(cmake_filename, "w") as f:
            f.write(textwrap.dedent(cmake_content))

        cmake_binary_dir = install_test_helper.create_temporary_dir("build")

        subprocess.check_call(["cmake", cmake_source_dir],
                              cwd=cmake_binary_dir)
        subprocess.check_call("make", cwd=cmake_binary_dir)


if __name__ == '__main__':
    unittest.main()

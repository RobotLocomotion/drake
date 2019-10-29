import os
import subprocess
import textwrap
import unittest

import install_test_helper


class FindPackageDrakeInstallTest(unittest.TestCase):
    def test_find_package_drake(self):
        cmake_source_dir = install_test_helper.create_temporary_dir("src")

        cc_content_1 = """
            #include <drake/common/symbolic.h>
            int main() {
              drake::symbolic::Environment environment;
              return 0;
            }
        """

        cc_filename_1 = os.path.join(cmake_source_dir, "main_1.cc")

        with open(cc_filename_1, "w") as f:
            f.write(textwrap.dedent(cc_content_1))

        cc_content_2 = """
            #include <drake/common/text_logging_gflags.h>
            int main() {
              drake::logging::HandleSpdlogGflags();
              return 0;
            }
        """

        cc_filename_2 = os.path.join(cmake_source_dir, "main_2.cc")

        with open(cc_filename_2, "w") as f:
            f.write(textwrap.dedent(cc_content_2))

        cmake_prefix_path = install_test_helper.get_install_dir()

        cmake_content = """
            cmake_minimum_required(VERSION 3.10)
            project(find_package_drake_install_test)
            set(CMAKE_PREFIX_PATH {cmake_prefix_path})
            find_package(drake CONFIG REQUIRED)
            add_executable(main_1 main_1.cc)
            target_link_libraries(main_1 drake::drake)
            add_executable(main_2 main_2.cc)
            target_link_libraries(main_2
              drake::drake-common-text-logging-gflags)
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

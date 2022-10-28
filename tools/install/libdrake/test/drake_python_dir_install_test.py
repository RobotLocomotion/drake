import os
import subprocess
import sys
import textwrap
import unittest

import install_test_helper


class DrakePythonDirInstallTest(unittest.TestCase):
    def test_drake_python_dir(self):
        cmake_source_dir = install_test_helper.create_temporary_dir("pydir_src")

        cmake_prefix_path = install_test_helper.get_install_dir()

        cmake_content = """
            cmake_minimum_required(VERSION 3.10)
            project(drake_python_dir_install_test)
            set(CMAKE_PREFIX_PATH {cmake_prefix_path})
            find_package(drake CONFIG REQUIRED)

            # Sanity check
            execute_process(
              COMMAND ${{CMAKE_COMMAND}} -E env PYTHONPATH=""
                {python_exe} -c "import pydrake"
              ERROR_QUIET
              RESULT_VARIABLE _IMPORT_RESULT)
            if(0 EQUAL _IMPORT_RESULT)
              message(FATAL_ERROR "Import of pydrake should have failed")
            endif()

            # Actual test
            execute_process(
              COMMAND ${{CMAKE_COMMAND}} -E env PYTHONPATH=${{drake_PYTHON_DIR}}
                {python_exe} -c "import pydrake"
              RESULT_VARIABLE _IMPORT_RESULT)
            if(NOT 0 EQUAL _IMPORT_RESULT)
              message(FATAL_ERROR "Import of pydrake failed")
            else()
              message(STATUS "Import of pydrake works as expected")
            endif()
        """.format(
            cmake_prefix_path=cmake_prefix_path, python_exe=sys.executable)

        cmake_filename = os.path.join(cmake_source_dir, "CMakeLists.txt")

        with open(cmake_filename, "w") as f:
            f.write(textwrap.dedent(cmake_content))

        cmake_binary_dir = install_test_helper.create_temporary_dir("pydir_build")

        subprocess.check_call(["cmake", cmake_source_dir],
                              cwd=cmake_binary_dir)


if __name__ == '__main__':
    unittest.main()

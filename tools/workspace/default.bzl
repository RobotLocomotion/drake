# -*- python -*-

load("@drake//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
load("@drake//tools/workspace:os.bzl", "os_repository")
load("@drake//tools/workspace/bazel_skylib:repository.bzl", "bazel_skylib_repository")  # noqa
load("@drake//tools/workspace/blas:repository.bzl", "blas_repository")
load("@drake//tools/workspace/boost:repository.bzl", "boost_repository")
load("@drake//tools/workspace/buildifier:repository.bzl", "buildifier_repository")  # noqa
load("@drake//tools/workspace/cc:repository.bzl", "cc_repository")
load("@drake//tools/workspace/ccd:repository.bzl", "ccd_repository")
load("@drake//tools/workspace/cds:repository.bzl", "cds_repository")
load("@drake//tools/workspace/clang_cindex_python3:repository.bzl", "clang_cindex_python3_repository")  # noqa
load("@drake//tools/workspace/clp:repository.bzl", "clp_repository")
load("@drake//tools/workspace/com_jidesoft_jide_oss:repository.bzl", "com_jidesoft_jide_oss_repository")  # noqa
load("@drake//tools/workspace/common_robotics_utilities:repository.bzl", "common_robotics_utilities_repository")  # noqa
load("@drake//tools/workspace/commons_io:repository.bzl", "commons_io_repository")  # noqa
load("@drake//tools/workspace/conex:repository.bzl", "conex_repository")
load("@drake//tools/workspace/csdp:repository.bzl", "csdp_repository")
load("@drake//tools/workspace/double_conversion:repository.bzl", "double_conversion_repository")  # noqa
load("@drake//tools/workspace/doxygen:repository.bzl", "doxygen_repository")
load("@drake//tools/workspace/drake_visualizer:repository.bzl", "drake_visualizer_repository")  # noqa
load("@drake//tools/workspace/dreal:repository.bzl", "dreal_repository")
load("@drake//tools/workspace/eigen:repository.bzl", "eigen_repository")
load("@drake//tools/workspace/expat:repository.bzl", "expat_repository")
load("@drake//tools/workspace/fcl:repository.bzl", "fcl_repository")
load("@drake//tools/workspace/fmt:repository.bzl", "fmt_repository")
load("@drake//tools/workspace/gflags:repository.bzl", "gflags_repository")
load("@drake//tools/workspace/gfortran:repository.bzl", "gfortran_repository")
load("@drake//tools/workspace/ghc_filesystem:repository.bzl", "ghc_filesystem_repository")  # noqa
load("@drake//tools/workspace/github3_py:repository.bzl", "github3_py_repository")  # noqa
load("@drake//tools/workspace/glew:repository.bzl", "glew_repository")
load("@drake//tools/workspace/glib:repository.bzl", "glib_repository")
load("@drake//tools/workspace/glx:repository.bzl", "glx_repository")
load("@drake//tools/workspace/googlebenchmark:repository.bzl", "googlebenchmark_repository")  # noqa
load("@drake//tools/workspace/gtest:repository.bzl", "gtest_repository")
load("@drake//tools/workspace/gurobi:repository.bzl", "gurobi_repository")
load("@drake//tools/workspace/ibex:repository.bzl", "ibex_repository")
load("@drake//tools/workspace/ignition_math:repository.bzl", "ignition_math_repository")  # noqa
load("@drake//tools/workspace/ignition_utils:repository.bzl", "ignition_utils_repository")  # noqa
load("@drake//tools/workspace/intel_realsense_ros:repository.bzl", "intel_realsense_ros_repository")  # noqa
load("@drake//tools/workspace/ipopt:repository.bzl", "ipopt_repository")
load("@drake//tools/workspace/lapack:repository.bzl", "lapack_repository")
load("@drake//tools/workspace/lcm:repository.bzl", "lcm_repository")
load("@drake//tools/workspace/libblas:repository.bzl", "libblas_repository")
load("@drake//tools/workspace/libjpeg:repository.bzl", "libjpeg_repository")
load("@drake//tools/workspace/liblapack:repository.bzl", "liblapack_repository")  # noqa
load("@drake//tools/workspace/liblz4:repository.bzl", "liblz4_repository")
load("@drake//tools/workspace/liblzma:repository.bzl", "liblzma_repository")
load("@drake//tools/workspace/libpng:repository.bzl", "libpng_repository")
load("@drake//tools/workspace/libtiff:repository.bzl", "libtiff_repository")
load("@drake//tools/workspace/meshcat:repository.bzl", "meshcat_repository")
load("@drake//tools/workspace/meshcat_python:repository.bzl", "meshcat_python_repository")  # noqa
load("@drake//tools/workspace/models:repository.bzl", "models_repository")
load("@drake//tools/workspace/mosek:repository.bzl", "mosek_repository")
load("@drake//tools/workspace/msgpack:repository.bzl", "msgpack_repository")
load("@drake//tools/workspace/net_sf_jchart2d:repository.bzl", "net_sf_jchart2d_repository")  # noqa
load("@drake//tools/workspace/nlopt:repository.bzl", "nlopt_repository")
load("@drake//tools/workspace/openblas:repository.bzl", "openblas_repository")
load("@drake//tools/workspace/opencl:repository.bzl", "opencl_repository")
load("@drake//tools/workspace/opengl:repository.bzl", "opengl_repository")
load("@drake//tools/workspace/optitrack_driver:repository.bzl", "optitrack_driver_repository")  # noqa
load("@drake//tools/workspace/org_apache_xmlgraphics_commons:repository.bzl", "org_apache_xmlgraphics_commons_repository")  # noqa
load("@drake//tools/workspace/osqp:repository.bzl", "osqp_repository")
load("@drake//tools/workspace/petsc:repository.bzl", "petsc_repository")
load("@drake//tools/workspace/picosat:repository.bzl", "picosat_repository")
load("@drake//tools/workspace/pybind11:repository.bzl", "pybind11_repository")
load("@drake//tools/workspace/pycodestyle:repository.bzl", "pycodestyle_repository")  # noqa
load("@drake//tools/workspace/pygame_py:repository.bzl", "pygame_py_repository")  # noqa
load("@drake//tools/workspace/python:repository.bzl", "python_repository")
load("@drake//tools/workspace/qdldl:repository.bzl", "qdldl_repository")
load("@drake//tools/workspace/ros_xacro:repository.bzl", "ros_xacro_repository")  # noqa
load("@drake//tools/workspace/rules_pkg:repository.bzl", "rules_pkg_repository")  # noqa
load("@drake//tools/workspace/rules_python:repository.bzl", "rules_python_repository")  # noqa
load("@drake//tools/workspace/scs:repository.bzl", "scs_repository")
load("@drake//tools/workspace/sdformat:repository.bzl", "sdformat_repository")
load("@drake//tools/workspace/snopt:repository.bzl", "snopt_repository")
load("@drake//tools/workspace/spdlog:repository.bzl", "spdlog_repository")
load("@drake//tools/workspace/stduuid:repository.bzl", "stduuid_repository")
load("@drake//tools/workspace/styleguide:repository.bzl", "styleguide_repository")  # noqa
load("@drake//tools/workspace/suitesparse:repository.bzl", "suitesparse_repository")  # noqa
load("@drake//tools/workspace/tinyobjloader:repository.bzl", "tinyobjloader_repository")  # noqa
load("@drake//tools/workspace/tinyxml2:repository.bzl", "tinyxml2_repository")
load("@drake//tools/workspace/uritemplate_py:repository.bzl", "uritemplate_py_repository")  # noqa
load("@drake//tools/workspace/usockets:repository.bzl", "usockets_repository")  # noqa
load("@drake//tools/workspace/uwebsockets:repository.bzl", "uwebsockets_repository")  # noqa
load("@drake//tools/workspace/voxelized_geometry_tools:repository.bzl", "voxelized_geometry_tools_repository")  # noqa
load("@drake//tools/workspace/vtk:repository.bzl", "vtk_repository")
load("@drake//tools/workspace/x11:repository.bzl", "x11_repository")
load("@drake//tools/workspace/xmlrunner_py:repository.bzl", "xmlrunner_py_repository")  # noqa
load("@drake//tools/workspace/yaml_cpp:repository.bzl", "yaml_cpp_repository")
load("@drake//tools/workspace/zlib:repository.bzl", "zlib_repository")

def add_default_repositories(excludes = [], mirrors = DEFAULT_MIRRORS):
    """Declares workspace repositories for all externals needed by drake (other
    than those built into Bazel, of course).  This is intended to be loaded and
    called from a WORKSPACE file.

    Args:
        excludes: list of string names of repositories to exclude; this can
          be useful if a WORKSPACE file has already supplied its own external
          of a given name.
    """
    if "bazel_skylib" not in excludes:
        bazel_skylib_repository(name = "bazel_skylib", mirrors = mirrors)
    if "blas" not in excludes:
        blas_repository(name = "blas")
    if "boost" not in excludes:
        boost_repository(name = "boost")
    if "buildifier" not in excludes:
        buildifier_repository(name = "buildifier", mirrors = mirrors)
    if "cc" not in excludes:
        cc_repository(name = "cc")
    if "ccd" not in excludes:
        ccd_repository(name = "ccd", mirrors = mirrors)
    if "cds" not in excludes:
        cds_repository(name = "cds", mirrors = mirrors)
    if "clang_cindex_python3" not in excludes:
        clang_cindex_python3_repository(name = "clang_cindex_python3", mirrors = mirrors)  # noqa
    if "clp" not in excludes:
        clp_repository(name = "clp")
    if "com_jidesoft_jide_oss" not in excludes:
        com_jidesoft_jide_oss_repository(name = "com_jidesoft_jide_oss", mirrors = mirrors)  # noqa
    if "common_robotics_utilities" not in excludes:
        common_robotics_utilities_repository(name = "common_robotics_utilities", mirrors = mirrors)  # noqa
    if "commons_io" not in excludes:
        commons_io_repository(name = "commons_io", mirrors = mirrors)
    if "conex" not in excludes:
        conex_repository(name = "conex", mirrors = mirrors)
    if "csdp" not in excludes:
        csdp_repository(name = "csdp", mirrors = mirrors)
    if "double_conversion" not in excludes:
        double_conversion_repository(name = "double_conversion")
    if "doxygen" not in excludes:
        doxygen_repository(name = "doxygen", mirrors = mirrors)
    if "drake_detected_os" not in excludes:
        os_repository(name = "drake_detected_os")
    if "drake_visualizer" not in excludes:
        drake_visualizer_repository(name = "drake_visualizer", mirrors = mirrors)  # noqa
    if "dreal" not in excludes:
        dreal_repository(name = "dreal", mirrors = mirrors)
    if "eigen" not in excludes:
        eigen_repository(name = "eigen")
    if "expat" not in excludes:
        expat_repository(name = "expat")
    if "fcl" not in excludes:
        fcl_repository(name = "fcl", mirrors = mirrors)
    if "fmt" not in excludes:
        fmt_repository(name = "fmt", mirrors = mirrors)
    if "gflags" not in excludes:
        gflags_repository(name = "gflags")
    if "gfortran" not in excludes:
        gfortran_repository(name = "gfortran")
    if "ghc_filesystem" not in excludes:
        ghc_filesystem_repository(name = "ghc_filesystem", mirrors = mirrors)
    if "github3_py" not in excludes:
        github3_py_repository(name = "github3_py", mirrors = mirrors)
    if "glew" not in excludes:
        glew_repository(name = "glew")
    if "glib" not in excludes:
        glib_repository(name = "glib")
    if "glx" not in excludes:
        glx_repository(name = "glx")
    if "googlebenchmark" not in excludes:
        googlebenchmark_repository(name = "googlebenchmark", mirrors = mirrors)
    if "gtest" not in excludes:
        gtest_repository(name = "gtest", mirrors = mirrors)
    if "gurobi" not in excludes:
        gurobi_repository(name = "gurobi")
    if "ibex" not in excludes:
        ibex_repository(name = "ibex", mirrors = mirrors)
    if "ignition_math" not in excludes:
        ignition_math_repository(name = "ignition_math", mirrors = mirrors)
    if "ignition_utils" not in excludes:
        ignition_utils_repository(name = "ignition_utils", mirrors = mirrors)
    if "intel_realsense_ros" not in excludes:
        intel_realsense_ros_repository(name = "intel_realsense_ros", mirrors = mirrors)  # noqa
    if "ipopt" not in excludes:
        ipopt_repository(name = "ipopt")
    if "lapack" not in excludes:
        lapack_repository(name = "lapack")
    if "lcm" not in excludes:
        lcm_repository(name = "lcm", mirrors = mirrors)
    if "libblas" not in excludes:
        libblas_repository(name = "libblas")
    if "libjpeg" not in excludes:
        libjpeg_repository(name = "libjpeg")
    if "liblapack" not in excludes:
        liblapack_repository(name = "liblapack")
    if "liblz4" not in excludes:
        liblz4_repository(name = "liblz4")
    if "liblzma" not in excludes:
        liblzma_repository(name = "liblzma")
    if "libpng" not in excludes:
        libpng_repository(name = "libpng")
    if "libtiff" not in excludes:
        libtiff_repository(name = "libtiff")
    if "meshcat" not in excludes:
        meshcat_repository(name = "meshcat", mirrors = mirrors)
    if "meshcat_python" not in excludes:
        meshcat_python_repository(name = "meshcat_python", mirrors = mirrors)
    if "models" not in excludes:
        models_repository(name = "models", mirrors = mirrors)
    if "mosek" not in excludes:
        mosek_repository(name = "mosek")
    if "msgpack" not in excludes:
        msgpack_repository(name = "msgpack")
    if "net_sf_jchart2d" not in excludes:
        net_sf_jchart2d_repository(name = "net_sf_jchart2d", mirrors = mirrors)
    if "nlopt" not in excludes:
        nlopt_repository(name = "nlopt")
    if "openblas" not in excludes:
        openblas_repository(name = "openblas")
    if "opencl" not in excludes:
        opencl_repository(name = "opencl")
    if "opengl" not in excludes:
        opengl_repository(name = "opengl")
    if "optitrack_driver" not in excludes:
        optitrack_driver_repository(name = "optitrack_driver", mirrors = mirrors)  # noqa
    if "org_apache_xmlgraphics_commons" not in excludes:
        org_apache_xmlgraphics_commons_repository(name = "org_apache_xmlgraphics_commons", mirrors = mirrors)  # noqa
    if "osqp" not in excludes:
        osqp_repository(name = "osqp", mirrors = mirrors)
    if "petsc" not in excludes:
        petsc_repository(name = "petsc", mirrors = mirrors)
    if "picosat" not in excludes:
        picosat_repository(name = "picosat", mirrors = mirrors)
    if "pybind11" not in excludes:
        pybind11_repository(name = "pybind11", mirrors = mirrors)
    if "pycodestyle" not in excludes:
        pycodestyle_repository(name = "pycodestyle", mirrors = mirrors)
    if "pygame_py" not in excludes:
        pygame_py_repository(name = "pygame_py", mirrors = mirrors)
    if "python" not in excludes:
        python_repository(name = "python")
    if "qdldl" not in excludes:
        qdldl_repository(name = "qdldl", mirrors = mirrors)
    if "ros_xacro" not in excludes:
        ros_xacro_repository(name = "ros_xacro", mirrors = mirrors)
    if "rules_pkg" not in excludes:
        rules_pkg_repository(name = "rules_pkg", mirrors = mirrors)
    if "rules_python" not in excludes:
        rules_python_repository(name = "rules_python", mirrors = mirrors)
    if "scs" not in excludes:
        scs_repository(name = "scs", mirrors = mirrors)
    if "sdformat" not in excludes:
        sdformat_repository(name = "sdformat", mirrors = mirrors)
    if "snopt" not in excludes:
        snopt_repository(name = "snopt")
    if "spdlog" not in excludes:
        spdlog_repository(name = "spdlog", mirrors = mirrors)
    if "stduuid" not in excludes:
        stduuid_repository(name = "stduuid", mirrors = mirrors)
    if "styleguide" not in excludes:
        styleguide_repository(name = "styleguide", mirrors = mirrors)
    if "suitesparse" not in excludes:
        suitesparse_repository(name = "suitesparse")
    if "tinyobjloader" not in excludes:
        tinyobjloader_repository(name = "tinyobjloader", mirrors = mirrors)
    if "tinyxml2" not in excludes:
        tinyxml2_repository(name = "tinyxml2")
    if "uritemplate_py" not in excludes:
        uritemplate_py_repository(name = "uritemplate_py", mirrors = mirrors)
    if "usockets" not in excludes:
        usockets_repository(name = "usockets", mirrors = mirrors)
    if "uwebsockets" not in excludes:
        uwebsockets_repository(name = "uwebsockets", mirrors = mirrors)
    if "voxelized_geometry_tools" not in excludes:
        voxelized_geometry_tools_repository(name = "voxelized_geometry_tools", mirrors = mirrors)  # noqa
    if "vtk" not in excludes:
        vtk_repository(name = "vtk", mirrors = mirrors)
    if "x11" not in excludes:
        x11_repository(name = "x11")
    if "xmlrunner_py" not in excludes:
        xmlrunner_py_repository(name = "xmlrunner_py", mirrors = mirrors)
    if "yaml_cpp" not in excludes:
        yaml_cpp_repository(name = "yaml_cpp")
    if "zlib" not in excludes:
        zlib_repository(name = "zlib")

def add_default_toolchains(excludes = []):
    """Register toolchains for each language (e.g., "py") not explicitly
    excluded and/or not using an automatically generated toolchain.

    Args:
        excludes: List of languages for which a toolchain should not be
            registered.
    """

    if "py" not in excludes:
        # The Python debug toolchain on Linux is not loaded automatically, but
        # may be used by specifying the command line option
        # --extra_toolchains=//tools/py_toolchain:linux_dbg_toolchain
        native.register_toolchains(
            "@drake//tools/py_toolchain:linux_toolchain",
            "@drake//tools/py_toolchain:macos_toolchain",
        )

def add_default_workspace(
        repository_excludes = [],
        toolchain_excludes = [],
        mirrors = DEFAULT_MIRRORS):
    """Declare repositories in this WORKSPACE for each dependency of @drake
    (e.g., "eigen") that is not explicitly excluded, and register toolchains
    for each language (e.g., "py") not explicitly excluded and/or not using an
    automatically generated toolchain.

    Args:
        repository_excludes: List of repositories that should not be declared
            in this WORKSPACE.
        toolchain_excludes: List of languages for which a toolchain should not
            be registered.
        mirrors: Dictionary of mirrors from which to download repository files.
            See mirrors.bzl file in this directory for the file format and
            default values.
    """

    add_default_repositories(excludes = repository_excludes, mirrors = mirrors)
    add_default_toolchains(excludes = toolchain_excludes)

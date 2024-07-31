load("//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
load("//tools/workspace/abseil_cpp_internal:repository.bzl", "abseil_cpp_internal_repository")  # noqa
load("//tools/workspace/bazel_skylib:repository.bzl", "bazel_skylib_repository")  # noqa
load("//tools/workspace/bazelisk:repository.bzl", "bazelisk_repository")
load("//tools/workspace/blas:repository.bzl", "blas_repository")
load("//tools/workspace/build_bazel_apple_support:repository.bzl", "build_bazel_apple_support_repository")  # noqa
load("//tools/workspace/buildifier:repository.bzl", "buildifier_repository")
load("//tools/workspace/cc:repository.bzl", "cc_repository")
load("//tools/workspace/ccd_internal:repository.bzl", "ccd_internal_repository")  # noqa
load("//tools/workspace/clang_cindex_python3_internal:repository.bzl", "clang_cindex_python3_internal_repository")  # noqa
load("//tools/workspace/clarabel_cpp_internal:repository.bzl", "clarabel_cpp_internal_repository")  # noqa
load("//tools/workspace/clp_internal:repository.bzl", "clp_internal_repository")  # noqa
load("//tools/workspace/coinutils_internal:repository.bzl", "coinutils_internal_repository")  # noqa
load("//tools/workspace/com_jidesoft_jide_oss:repository.bzl", "com_jidesoft_jide_oss_repository")  # noqa
load("//tools/workspace/common_robotics_utilities:repository.bzl", "common_robotics_utilities_repository")  # noqa
load("//tools/workspace/commons_io:repository.bzl", "commons_io_repository")
load("//tools/workspace/conex_internal:repository.bzl", "conex_internal_repository")  # noqa
load("//tools/workspace/crate_universe:repository.bzl", "crate_universe_repositories")  # noqa
load("//tools/workspace/csdp_internal:repository.bzl", "csdp_internal_repository")  # noqa
load("//tools/workspace/curl_internal:repository.bzl", "curl_internal_repository")  # noqa
load("//tools/workspace/dm_control_internal:repository.bzl", "dm_control_internal_repository")  # noqa
load("//tools/workspace/doxygen:repository.bzl", "doxygen_repository")
load("//tools/workspace/drake_models:repository.bzl", "drake_models_repository")  # noqa
load("//tools/workspace/eigen:repository.bzl", "eigen_repository")
load("//tools/workspace/fcl_internal:repository.bzl", "fcl_internal_repository")  # noqa
load("//tools/workspace/fmt:repository.bzl", "fmt_repository")
load("//tools/workspace/gflags:repository.bzl", "gflags_repository")
load("//tools/workspace/gfortran:repository.bzl", "gfortran_repository")
load("//tools/workspace/github3_py_internal:repository.bzl", "github3_py_internal_repository")  # noqa
load("//tools/workspace/glib:repository.bzl", "glib_repository")
load("//tools/workspace/glx:repository.bzl", "glx_repository")
load("//tools/workspace/googlebenchmark:repository.bzl", "googlebenchmark_repository")  # noqa
load("//tools/workspace/gtest:repository.bzl", "gtest_repository")
load("//tools/workspace/gurobi:repository.bzl", "gurobi_repository")
load("//tools/workspace/gymnasium_py:repository.bzl", "gymnasium_py_repository")  # noqa
load("//tools/workspace/gz_math_internal:repository.bzl", "gz_math_internal_repository")  # noqa
load("//tools/workspace/gz_utils_internal:repository.bzl", "gz_utils_internal_repository")  # noqa
load("//tools/workspace/highway_internal:repository.bzl", "highway_internal_repository")  # noqa
load("//tools/workspace/ipopt:repository.bzl", "ipopt_repository")
load("//tools/workspace/ipopt_internal_fromsource:repository.bzl", "ipopt_internal_fromsource_repository")  # noqa
load("//tools/workspace/ipopt_internal_pkgconfig:repository.bzl", "ipopt_internal_pkgconfig_repository")  # noqa
load("//tools/workspace/lapack:repository.bzl", "lapack_repository")
load("//tools/workspace/lcm:repository.bzl", "lcm_repository")
load("//tools/workspace/libblas:repository.bzl", "libblas_repository")
load("//tools/workspace/libjpeg_turbo_internal:repository.bzl", "libjpeg_turbo_internal_repository")  # noqa
load("//tools/workspace/liblapack:repository.bzl", "liblapack_repository")
load("//tools/workspace/libpfm:repository.bzl", "libpfm_repository")
load("//tools/workspace/libpng_internal:repository.bzl", "libpng_internal_repository")  # noqa
load("//tools/workspace/libtiff_internal:repository.bzl", "libtiff_internal_repository")  # noqa
load("//tools/workspace/meshcat:repository.bzl", "meshcat_repository")
load("//tools/workspace/mosek:repository.bzl", "mosek_repository")
load("//tools/workspace/mpmath_py_internal:repository.bzl", "mpmath_py_internal_repository")  # noqa
load("//tools/workspace/msgpack_internal:repository.bzl", "msgpack_internal_repository")  # noqa
load("//tools/workspace/mujoco_menagerie_internal:repository.bzl", "mujoco_menagerie_internal_repository")  # noqa
load("//tools/workspace/mumps_internal:repository.bzl", "mumps_internal_repository")  # noqa
load("//tools/workspace/mypy_extensions_internal:repository.bzl", "mypy_extensions_internal_repository")  # noqa
load("//tools/workspace/mypy_internal:repository.bzl", "mypy_internal_repository")  # noqa
load("//tools/workspace/nanoflann_internal:repository.bzl", "nanoflann_internal_repository")  # noqa
load("//tools/workspace/nasm:repository.bzl", "nasm_repository")
load("//tools/workspace/net_sf_jchart2d:repository.bzl", "net_sf_jchart2d_repository")  # noqa
load("//tools/workspace/nlohmann_internal:repository.bzl", "nlohmann_internal_repository")  # noqa
load("//tools/workspace/nlopt_internal:repository.bzl", "nlopt_internal_repository")  # noqa
load("//tools/workspace/onetbb_internal:repository.bzl", "onetbb_internal_repository")  # noqa
load("//tools/workspace/openblas:repository.bzl", "openblas_repository")
load("//tools/workspace/opencl:repository.bzl", "opencl_repository")
load("//tools/workspace/opengl:repository.bzl", "opengl_repository")
load("//tools/workspace/openusd_internal:repository.bzl", "openusd_internal_repository")  # noqa
load("//tools/workspace/org_apache_xmlgraphics_commons:repository.bzl", "org_apache_xmlgraphics_commons_repository")  # noqa
load("//tools/workspace/osqp_internal:repository.bzl", "osqp_internal_repository")  # noqa
load("//tools/workspace/picosha2_internal:repository.bzl", "picosha2_internal_repository")  # noqa
load("//tools/workspace/platforms:repository.bzl", "platforms_repository")
load("//tools/workspace/pybind11:repository.bzl", "pybind11_repository")
load("//tools/workspace/pycodestyle:repository.bzl", "pycodestyle_repository")
load("//tools/workspace/python:repository.bzl", "python_repository")
load("//tools/workspace/qdldl_internal:repository.bzl", "qdldl_internal_repository")  # noqa
load("//tools/workspace/qhull_internal:repository.bzl", "qhull_internal_repository")  # noqa
load("//tools/workspace/ros_xacro_internal:repository.bzl", "ros_xacro_internal_repository")  # noqa
load("//tools/workspace/rules_cc:repository.bzl", "rules_cc_repository")  # noqa
load("//tools/workspace/rules_license:repository.bzl", "rules_license_repository")  # noqa
load("//tools/workspace/rules_python:repository.bzl", "rules_python_repository")  # noqa
load("//tools/workspace/rules_rust:repository.bzl", "rules_rust_repository")
load("//tools/workspace/rules_rust_tinyjson:repository.bzl", "rules_rust_tinyjson_repository")  # noqa
load("//tools/workspace/rust_toolchain:repository.bzl", "register_rust_toolchains", "rust_toolchain_repositories")  # noqa
load("//tools/workspace/scs_internal:repository.bzl", "scs_internal_repository")  # noqa
load("//tools/workspace/sdformat_internal:repository.bzl", "sdformat_internal_repository")  # noqa
load("//tools/workspace/snopt:repository.bzl", "snopt_repository")
load("//tools/workspace/spdlog:repository.bzl", "spdlog_repository")
load("//tools/workspace/stable_baselines3_internal:repository.bzl", "stable_baselines3_internal_repository")  # noqa
load("//tools/workspace/statsjs:repository.bzl", "statsjs_repository")
load("//tools/workspace/stduuid_internal:repository.bzl", "stduuid_internal_repository")  # noqa
load("//tools/workspace/styleguide:repository.bzl", "styleguide_repository")
load("//tools/workspace/suitesparse_internal:repository.bzl", "suitesparse_internal_repository")  # noqa
load("//tools/workspace/sympy_py_internal:repository.bzl", "sympy_py_internal_repository")  # noqa
load("//tools/workspace/tinygltf_internal:repository.bzl", "tinygltf_internal_repository")  # noqa
load("//tools/workspace/tinyobjloader_internal:repository.bzl", "tinyobjloader_internal_repository")  # noqa
load("//tools/workspace/tinyxml2_internal:repository.bzl", "tinyxml2_internal_repository")  # noqa
load("//tools/workspace/tomli_internal:repository.bzl", "tomli_internal_repository")  # noqa
load("//tools/workspace/typing_extensions_internal:repository.bzl", "typing_extensions_internal_repository")  # noqa
load("//tools/workspace/uritemplate_py_internal:repository.bzl", "uritemplate_py_internal_repository")  # noqa
load("//tools/workspace/usockets_internal:repository.bzl", "usockets_internal_repository")  # noqa
load("//tools/workspace/uwebsockets_internal:repository.bzl", "uwebsockets_internal_repository")  # noqa
load("//tools/workspace/voxelized_geometry_tools:repository.bzl", "voxelized_geometry_tools_repository")  # noqa
load("//tools/workspace/vtk_internal:repository.bzl", "vtk_internal_repository")  # noqa
load("//tools/workspace/x11:repository.bzl", "x11_repository")
load("//tools/workspace/xmlrunner_py:repository.bzl", "xmlrunner_py_repository")  # noqa
load("//tools/workspace/yaml_cpp_internal:repository.bzl", "yaml_cpp_internal_repository")  # noqa
load("//tools/workspace/zlib:repository.bzl", "zlib_repository")

def add_default_repositories(excludes = [], mirrors = DEFAULT_MIRRORS):
    """Declares workspace repositories for all externals needed by drake (other
    than those built into Bazel, of course).  This is intended to be loaded and
    called from a WORKSPACE file.

    Args:
        excludes: list of string names of repositories to exclude; this can
          be useful if a WORKSPACE file has already supplied its own external
          of a given name.
    """
    if "abseil_cpp_internal" not in excludes:
        abseil_cpp_internal_repository(name = "abseil_cpp_internal", mirrors = mirrors)  # noqa
    if "bazelisk" not in excludes:
        bazelisk_repository(name = "bazelisk", mirrors = mirrors)
    if "bazel_skylib" not in excludes:
        bazel_skylib_repository(name = "bazel_skylib", mirrors = mirrors)
    if "blas" not in excludes:
        blas_repository(name = "blas")
    if "build_bazel_apple_support" not in excludes:
        build_bazel_apple_support_repository(name = "build_bazel_apple_support", mirrors = mirrors)  # noqa
    if "buildifier" not in excludes:
        buildifier_repository(name = "buildifier", mirrors = mirrors)
    if "cc" not in excludes:
        cc_repository(name = "cc")
    if "ccd_internal" not in excludes:
        ccd_internal_repository(name = "ccd_internal", mirrors = mirrors)
    if "clang_cindex_python3_internal" not in excludes:
        clang_cindex_python3_internal_repository(name = "clang_cindex_python3_internal", mirrors = mirrors)  # noqa
    if "clarabel_cpp_internal" not in excludes:
        clarabel_cpp_internal_repository(name = "clarabel_cpp_internal", mirrors = mirrors)  # noqa
    if "clp_internal" not in excludes:
        clp_internal_repository(name = "clp_internal", mirrors = mirrors)
    if "coinutils_internal" not in excludes:
        coinutils_internal_repository(name = "coinutils_internal", mirrors = mirrors)  # noqa
    if "com_jidesoft_jide_oss" not in excludes:
        com_jidesoft_jide_oss_repository(name = "com_jidesoft_jide_oss", mirrors = mirrors)  # noqa
    if "common_robotics_utilities" not in excludes:
        common_robotics_utilities_repository(name = "common_robotics_utilities", mirrors = mirrors)  # noqa
    if "commons_io" not in excludes:
        commons_io_repository(name = "commons_io", mirrors = mirrors)
    if "conex_internal" not in excludes:
        conex_internal_repository(name = "conex_internal", mirrors = mirrors)
    if "crate_universe" not in excludes:
        crate_universe_repositories(mirrors = mirrors, excludes = excludes)
    if "csdp_internal" not in excludes:
        csdp_internal_repository(name = "csdp_internal", mirrors = mirrors)
    if "curl_internal" not in excludes:
        curl_internal_repository(name = "curl_internal", mirrors = mirrors)
    if "doxygen" not in excludes:
        doxygen_repository(name = "doxygen", mirrors = mirrors)
    if "dm_control_internal" not in excludes:
        dm_control_internal_repository(name = "dm_control_internal", mirrors = mirrors)  # noqa
    if "drake_models" not in excludes:
        drake_models_repository(name = "drake_models", mirrors = mirrors)
    if "eigen" not in excludes:
        eigen_repository(name = "eigen")
    if "fcl_internal" not in excludes:
        fcl_internal_repository(name = "fcl_internal", mirrors = mirrors)
    if "fmt" not in excludes:
        fmt_repository(name = "fmt", mirrors = mirrors)
    if "gflags" not in excludes:
        gflags_repository(name = "gflags", mirrors = mirrors)
    if "gfortran" not in excludes:
        gfortran_repository(name = "gfortran")
    if "github3_py_internal" not in excludes:
        github3_py_internal_repository(name = "github3_py_internal", mirrors = mirrors)  # noqa
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
    if "gz_math_internal" not in excludes:
        gz_math_internal_repository(name = "gz_math_internal", mirrors = mirrors)  # noqa
    if "gz_utils_internal" not in excludes:
        gz_utils_internal_repository(name = "gz_utils_internal", mirrors = mirrors)  # noqa
    if "gymnasium_py" not in excludes:
        gymnasium_py_repository(name = "gymnasium_py", mirrors = mirrors)
    if "highway_internal" not in excludes:
        highway_internal_repository(name = "highway_internal", mirrors = mirrors)  # noqa
    if "ipopt" not in excludes:
        ipopt_repository(name = "ipopt")
    if "ipopt_internal_fromsource" not in excludes:
        ipopt_internal_fromsource_repository(name = "ipopt_internal_fromsource", mirrors = mirrors)  # noqa
    if "ipopt_internal_pkgconfig" not in excludes:
        ipopt_internal_pkgconfig_repository(name = "ipopt_internal_pkgconfig")
    if "lapack" not in excludes:
        lapack_repository(name = "lapack")
    if "lcm" not in excludes:
        lcm_repository(name = "lcm", mirrors = mirrors)
    if "libblas" not in excludes:
        libblas_repository(name = "libblas")
    if "libjpeg_turbo_internal" not in excludes:
        libjpeg_turbo_internal_repository(name = "libjpeg_turbo_internal", mirrors = mirrors)  # noqa
    if "liblapack" not in excludes:
        liblapack_repository(name = "liblapack")
    if "libpfm" not in excludes:
        libpfm_repository(name = "libpfm")
    if "libpng_internal" not in excludes:
        libpng_internal_repository(name = "libpng_internal", mirrors = mirrors)
    if "libtiff_internal" not in excludes:
        libtiff_internal_repository(name = "libtiff_internal", mirrors = mirrors)  # noqa
    if "meshcat" not in excludes:
        meshcat_repository(name = "meshcat", mirrors = mirrors)
    if "mosek" not in excludes:
        mosek_repository(name = "mosek", mirrors = mirrors)
    if "mpmath_py_internal" not in excludes:
        mpmath_py_internal_repository(name = "mpmath_py_internal", mirrors = mirrors)  # noqa
    if "msgpack_internal" not in excludes:
        msgpack_internal_repository(name = "msgpack_internal", mirrors = mirrors)  # noqa
    if "mujoco_menagerie_internal" not in excludes:
        mujoco_menagerie_internal_repository(name = "mujoco_menagerie_internal", mirrors = mirrors)  # noqa
    if "mumps_internal" not in excludes:
        mumps_internal_repository(name = "mumps_internal")
    if "mypy_extensions_internal" not in excludes:
        mypy_extensions_internal_repository(name = "mypy_extensions_internal", mirrors = mirrors)  # noqa
    if "mypy_internal" not in excludes:
        mypy_internal_repository(name = "mypy_internal", mirrors = mirrors)
    if "nanoflann_internal" not in excludes:
        nanoflann_internal_repository(name = "nanoflann_internal", mirrors = mirrors)  # noqa
    if "nasm" not in excludes:
        nasm_repository(name = "nasm")
    if "net_sf_jchart2d" not in excludes:
        net_sf_jchart2d_repository(name = "net_sf_jchart2d", mirrors = mirrors)
    if "nlohmann_internal" not in excludes:
        nlohmann_internal_repository(name = "nlohmann_internal", mirrors = mirrors)  # noqa
    if "nlopt_internal" not in excludes:
        nlopt_internal_repository(name = "nlopt_internal", mirrors = mirrors)
    if "onetbb_internal" not in excludes:
        onetbb_internal_repository(name = "onetbb_internal", mirrors = mirrors)
    if "openblas" not in excludes:
        # The @openblas external is deprecated in Drake's WORKSPACE and will be
        # removed on or after 2024-09-01.
        openblas_repository(name = "openblas")
    if "opencl" not in excludes:
        opencl_repository(name = "opencl")
    if "opengl" not in excludes:
        opengl_repository(name = "opengl")
    if "openusd_internal" not in excludes:
        openusd_internal_repository(name = "openusd_internal", mirrors = mirrors)  # noqa
    if "org_apache_xmlgraphics_commons" not in excludes:
        org_apache_xmlgraphics_commons_repository(name = "org_apache_xmlgraphics_commons", mirrors = mirrors)  # noqa
    if "osqp_internal" not in excludes:
        osqp_internal_repository(name = "osqp_internal", mirrors = mirrors)
    if "picosha2_internal" not in excludes:
        picosha2_internal_repository(name = "picosha2_internal", mirrors = mirrors)  # noqa
    if "platforms" not in excludes:
        platforms_repository(name = "platforms", mirrors = mirrors)
    if "pybind11" not in excludes:
        pybind11_repository(name = "pybind11", mirrors = mirrors)
    if "pycodestyle" not in excludes:
        pycodestyle_repository(name = "pycodestyle", mirrors = mirrors)
    if "python" not in excludes:
        python_repository(name = "python")
    if "qdldl_internal" not in excludes:
        qdldl_internal_repository(name = "qdldl_internal", mirrors = mirrors)
    if "qhull_internal" not in excludes:
        qhull_internal_repository(name = "qhull_internal", mirrors = mirrors)
    if "ros_xacro_internal" not in excludes:
        ros_xacro_internal_repository(name = "ros_xacro_internal", mirrors = mirrors)  # noqa
    if "rules_cc" not in excludes:
        rules_cc_repository(name = "rules_cc", mirrors = mirrors)
    if "rules_license" not in excludes:
        rules_license_repository(name = "rules_license", mirrors = mirrors)
    if "rules_python" not in excludes:
        rules_python_repository(name = "rules_python", mirrors = mirrors)
    if "rules_rust" not in excludes:
        rules_rust_repository(name = "rules_rust", mirrors = mirrors)
    if "rules_rust_tinyjson" not in excludes:
        rules_rust_tinyjson_repository(name = "rules_rust_tinyjson", mirrors = mirrors)  # noqa
    if "rust_toolchain" not in excludes:
        rust_toolchain_repositories(mirrors = mirrors, excludes = excludes)
    if "scs_internal" not in excludes:
        scs_internal_repository(name = "scs_internal", mirrors = mirrors)
    if "sdformat_internal" not in excludes:
        sdformat_internal_repository(name = "sdformat_internal", mirrors = mirrors)  # noqa
    if "snopt" not in excludes:
        snopt_repository(name = "snopt")
    if "spdlog" not in excludes:
        spdlog_repository(name = "spdlog", mirrors = mirrors)
    if "stable_baselines3_internal" not in excludes:
        stable_baselines3_internal_repository(name = "stable_baselines3_internal", mirrors = mirrors)  # noqa
    if "statsjs" not in excludes:
        statsjs_repository(name = "statsjs", mirrors = mirrors)
    if "stduuid_internal" not in excludes:
        stduuid_internal_repository(name = "stduuid_internal", mirrors = mirrors)  # noqa
    if "styleguide" not in excludes:
        styleguide_repository(name = "styleguide", mirrors = mirrors)
    if "suitesparse_internal" not in excludes:
        suitesparse_internal_repository(name = "suitesparse_internal", mirrors = mirrors)  # noqa
    if "sympy_py_internal" not in excludes:
        sympy_py_internal_repository(name = "sympy_py_internal", mirrors = mirrors)  # noqa
    if "tinygltf_internal" not in excludes:
        tinygltf_internal_repository(name = "tinygltf_internal", mirrors = mirrors)  # noqa
    if "tinyobjloader_internal" not in excludes:
        tinyobjloader_internal_repository(name = "tinyobjloader_internal", mirrors = mirrors)  # noqa
    if "tinyxml2_internal" not in excludes:
        tinyxml2_internal_repository(name = "tinyxml2_internal", mirrors = mirrors)  # noqa
    if "tomli_internal" not in excludes:
        tomli_internal_repository(name = "tomli_internal", mirrors = mirrors)
    if "typing_extensions_internal" not in excludes:
        typing_extensions_internal_repository(name = "typing_extensions_internal", mirrors = mirrors)  # noqa
    if "uritemplate_py_internal" not in excludes:
        uritemplate_py_internal_repository(name = "uritemplate_py_internal", mirrors = mirrors)  # noqa
    if "usockets_internal" not in excludes:
        usockets_internal_repository(name = "usockets_internal", mirrors = mirrors)  # noqa
    if "uwebsockets_internal" not in excludes:
        uwebsockets_internal_repository(name = "uwebsockets_internal", mirrors = mirrors)  # noqa
    if "voxelized_geometry_tools" not in excludes:
        voxelized_geometry_tools_repository(name = "voxelized_geometry_tools", mirrors = mirrors)  # noqa
    if "vtk_internal" not in excludes:
        vtk_internal_repository(name = "vtk_internal", mirrors = mirrors)
    if "x11" not in excludes:
        x11_repository(name = "x11")
    if "xmlrunner_py" not in excludes:
        xmlrunner_py_repository(name = "xmlrunner_py", mirrors = mirrors)
    if "yaml_cpp_internal" not in excludes:
        yaml_cpp_internal_repository(name = "yaml_cpp_internal", mirrors = mirrors)  # noqa
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
        native.register_toolchains(
            "//tools/py_toolchain:toolchain",
        )
    if "rust" not in excludes:
        register_rust_toolchains()

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

load("//tools/workspace:alias.bzl", "alias_repository")
load("//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
load("//tools/workspace/abseil_cpp_internal:repository.bzl", "abseil_cpp_internal_repository")  # noqa
load("//tools/workspace/bazelisk_internal:repository.bzl", "bazelisk_internal_repository")  # noqa
load("//tools/workspace/blas:repository.bzl", "blas_repository")
load("//tools/workspace/buildifier:repository.bzl", "buildifier_repository")
load("//tools/workspace/ccd_internal:repository.bzl", "ccd_internal_repository")
load("//tools/workspace/clang_cindex_python3_internal:repository.bzl", "clang_cindex_python3_internal_repository")  # noqa
load("//tools/workspace/clarabel_cpp_internal:repository.bzl", "clarabel_cpp_internal_repository")  # noqa
load("//tools/workspace/clp_internal:repository.bzl", "clp_internal_repository")
load("//tools/workspace/coinutils_internal:repository.bzl", "coinutils_internal_repository")  # noqa
load("//tools/workspace/common_robotics_utilities_internal:repository.bzl", "common_robotics_utilities_internal_repository")  # noqa
load("//tools/workspace/cpplint_internal:repository.bzl", "cpplint_internal_repository")  # noqa
load("//tools/workspace/crate_universe:repository.bzl", "crate_universe_repositories_internal")  # noqa
load("//tools/workspace/csdp_internal:repository.bzl", "csdp_internal_repository")  # noqa
load("//tools/workspace/curl_internal:repository.bzl", "curl_internal_repository")  # noqa
load("//tools/workspace/dm_control_internal:repository.bzl", "dm_control_internal_repository")  # noqa
load("//tools/workspace/doxygen_internal:repository.bzl", "doxygen_internal_repository")  # noqa
load("//tools/workspace/drake_models:repository.bzl", "drake_models_repository")
load("//tools/workspace/fcl_internal:repository.bzl", "fcl_internal_repository")
load("//tools/workspace/gflags:repository.bzl", "gflags_repository")
load("//tools/workspace/gfortran_internal:repository.bzl", "gfortran_internal_repository")  # noqa
load("//tools/workspace/github3_py_internal:repository.bzl", "github3_py_internal_repository")  # noqa
load("//tools/workspace/gklib_internal:repository.bzl", "gklib_internal_repository")  # noqa
load("//tools/workspace/gtest:repository.bzl", "gtest_repository")
load("//tools/workspace/gurobi:repository.bzl", "gurobi_repository")
load("//tools/workspace/gymnasium_py_internal:repository.bzl", "gymnasium_py_internal_repository")  # noqa
load("//tools/workspace/gz_math_internal:repository.bzl", "gz_math_internal_repository")  # noqa
load("//tools/workspace/gz_utils_internal:repository.bzl", "gz_utils_internal_repository")  # noqa
load("//tools/workspace/highway_internal:repository.bzl", "highway_internal_repository")  # noqa
load("//tools/workspace/implib_so_internal:repository.bzl", "implib_so_internal_repository")  # noqa
load("//tools/workspace/ipopt_internal:repository.bzl", "ipopt_internal_repository")  # noqa
load("//tools/workspace/lapack:repository.bzl", "lapack_repository")
load("//tools/workspace/lapack_internal:repository.bzl", "lapack_internal_repository")  # noqa
load("//tools/workspace/lcm:repository.bzl", "lcm_repository")
load("//tools/workspace/libjpeg_turbo_internal:repository.bzl", "libjpeg_turbo_internal_repository")  # noqa
load("//tools/workspace/libpng_internal:repository.bzl", "libpng_internal_repository")  # noqa
load("//tools/workspace/libtiff_internal:repository.bzl", "libtiff_internal_repository")  # noqa
load("//tools/workspace/meshcat:repository.bzl", "meshcat_repository")
load("//tools/workspace/metis_internal:repository.bzl", "metis_internal_repository")  # noqa
load("//tools/workspace/mosek:repository.bzl", "mosek_repository")
load("//tools/workspace/mpmath_py_internal:repository.bzl", "mpmath_py_internal_repository")  # noqa
load("//tools/workspace/msgpack_internal:repository.bzl", "msgpack_internal_repository")  # noqa
load("//tools/workspace/mujoco_menagerie_internal:repository.bzl", "mujoco_menagerie_internal_repository")  # noqa
load("//tools/workspace/mypy_extensions_internal:repository.bzl", "mypy_extensions_internal_repository")  # noqa
load("//tools/workspace/mypy_internal:repository.bzl", "mypy_internal_repository")  # noqa
load("//tools/workspace/nanoflann_internal:repository.bzl", "nanoflann_internal_repository")  # noqa
load("//tools/workspace/nlohmann_internal:repository.bzl", "nlohmann_internal_repository")  # noqa
load("//tools/workspace/nlopt_internal:repository.bzl", "nlopt_internal_repository")  # noqa
load("//tools/workspace/onetbb_internal:repository.bzl", "onetbb_internal_repository")  # noqa
load("//tools/workspace/opencl:repository.bzl", "opencl_repository")
load("//tools/workspace/osqp_internal:repository.bzl", "osqp_internal_repository")  # noqa
load("//tools/workspace/pathspec_internal:repository.bzl", "pathspec_internal_repository")  # noqa
load("//tools/workspace/picosha2_internal:repository.bzl", "picosha2_internal_repository")  # noqa
load("//tools/workspace/pkgconfig_blas_internal:repository.bzl", "pkgconfig_blas_internal_repository")  # noqa
load("//tools/workspace/pkgconfig_lapack_internal:repository.bzl", "pkgconfig_lapack_internal_repository")  # noqa
load("//tools/workspace/poisson_disk_sampling_internal:repository.bzl", "poisson_disk_sampling_internal_repository")  # noqa
load("//tools/workspace/pybind11:repository.bzl", "pybind11_repository")
load("//tools/workspace/pycodestyle:repository.bzl", "pycodestyle_repository")
load("//tools/workspace/pycodestyle_internal:repository.bzl", "pycodestyle_internal_repository")  # noqa
load("//tools/workspace/python:repository.bzl", "python_repository")
load("//tools/workspace/qdldl_internal:repository.bzl", "qdldl_internal_repository")  # noqa
load("//tools/workspace/qhull_internal:repository.bzl", "qhull_internal_repository")  # noqa
load("//tools/workspace/ros_xacro_internal:repository.bzl", "ros_xacro_internal_repository")  # noqa
load("//tools/workspace/scs_internal:repository.bzl", "scs_internal_repository")
load("//tools/workspace/sdformat_internal:repository.bzl", "sdformat_internal_repository")  # noqa
load("//tools/workspace/snopt:repository.bzl", "snopt_repository")
load("//tools/workspace/spgrid_internal:repository.bzl", "spgrid_module_extension_impl")  # noqa
load("//tools/workspace/spral_internal:repository.bzl", "spral_internal_repository")  # noqa
load("//tools/workspace/stable_baselines3_internal:repository.bzl", "stable_baselines3_internal_repository")  # noqa
load("//tools/workspace/statsjs_internal:repository.bzl", "statsjs_internal_repository")  # noqa
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
load("//tools/workspace/voxelized_geometry_tools_internal:repository.bzl", "voxelized_geometry_tools_internal_repository")  # noqa
load("//tools/workspace/vtk_internal:repository.bzl", "vtk_internal_repository")
load("//tools/workspace/x11:repository.bzl", "x11_repository")
load("//tools/workspace/xmlrunner_py_internal:repository.bzl", "xmlrunner_py_internal_repository")  # noqa
load("//tools/workspace/yaml_cpp_internal:repository.bzl", "yaml_cpp_internal_repository")  # noqa

def _add_internal_repositories():
    """Adds repositories for non-public, repository rule externals."""

    mirrors = DEFAULT_MIRRORS
    abseil_cpp_internal_repository(name = "abseil_cpp_internal", mirrors = mirrors)  # noqa
    bazelisk_internal_repository(name = "bazelisk_internal", mirrors = mirrors)
    ccd_internal_repository(name = "ccd_internal", mirrors = mirrors)
    clang_cindex_python3_internal_repository(name = "clang_cindex_python3_internal", mirrors = mirrors)  # noqa
    clarabel_cpp_internal_repository(name = "clarabel_cpp_internal", mirrors = mirrors)  # noqa
    clp_internal_repository(name = "clp_internal", mirrors = mirrors)
    coinutils_internal_repository(name = "coinutils_internal", mirrors = mirrors)  # noqa
    common_robotics_utilities_internal_repository(name = "common_robotics_utilities_internal", mirrors = mirrors)  # noqa
    cpplint_internal_repository(name = "cpplint_internal", mirrors = mirrors)
    csdp_internal_repository(name = "csdp_internal", mirrors = mirrors)
    curl_internal_repository(name = "curl_internal", mirrors = mirrors)
    doxygen_internal_repository(name = "doxygen_internal", mirrors = mirrors)
    dm_control_internal_repository(name = "dm_control_internal", mirrors = mirrors)  # noqa
    fcl_internal_repository(name = "fcl_internal", mirrors = mirrors)
    gfortran_internal_repository(name = "gfortran_internal")
    github3_py_internal_repository(name = "github3_py_internal", mirrors = mirrors)  # noqa
    gklib_internal_repository(name = "gklib_internal", mirrors = mirrors)
    gz_math_internal_repository(name = "gz_math_internal", mirrors = mirrors)
    gz_utils_internal_repository(name = "gz_utils_internal", mirrors = mirrors)
    gymnasium_py_internal_repository(name = "gymnasium_py_internal", mirrors = mirrors)  # noqa
    highway_internal_repository(name = "highway_internal", mirrors = mirrors)
    implib_so_internal_repository(name = "implib_so_internal", mirrors = mirrors)  # noqa
    ipopt_internal_repository(name = "ipopt_internal", mirrors = mirrors)
    lapack_internal_repository(name = "lapack_internal", mirrors = mirrors)
    libjpeg_turbo_internal_repository(name = "libjpeg_turbo_internal", mirrors = mirrors)  # noqa
    libpng_internal_repository(name = "libpng_internal", mirrors = mirrors)
    libtiff_internal_repository(name = "libtiff_internal", mirrors = mirrors)
    metis_internal_repository(name = "metis_internal", mirrors = mirrors)
    mpmath_py_internal_repository(name = "mpmath_py_internal", mirrors = mirrors)  # noqa
    msgpack_internal_repository(name = "msgpack_internal", mirrors = mirrors)
    mujoco_menagerie_internal_repository(name = "mujoco_menagerie_internal", mirrors = mirrors)  # noqa
    mypy_extensions_internal_repository(name = "mypy_extensions_internal", mirrors = mirrors)  # noqa
    mypy_internal_repository(name = "mypy_internal", mirrors = mirrors)
    nanoflann_internal_repository(name = "nanoflann_internal", mirrors = mirrors)  # noqa
    nlohmann_internal_repository(name = "nlohmann_internal", mirrors = mirrors)
    nlopt_internal_repository(name = "nlopt_internal", mirrors = mirrors)
    onetbb_internal_repository(name = "onetbb_internal", mirrors = mirrors)
    osqp_internal_repository(name = "osqp_internal", mirrors = mirrors)
    pathspec_internal_repository(name = "pathspec_internal", mirrors = mirrors)
    picosha2_internal_repository(name = "picosha2_internal", mirrors = mirrors)
    pkgconfig_blas_internal_repository(name = "pkgconfig_blas_internal")
    pkgconfig_lapack_internal_repository(name = "pkgconfig_lapack_internal")
    poisson_disk_sampling_internal_repository(name = "poisson_disk_sampling_internal", mirrors = mirrors)  # noqa
    pycodestyle_internal_repository(name = "pycodestyle_internal", mirrors = mirrors)  # noqa
    qdldl_internal_repository(name = "qdldl_internal", mirrors = mirrors)
    qhull_internal_repository(name = "qhull_internal", mirrors = mirrors)
    ros_xacro_internal_repository(name = "ros_xacro_internal", mirrors = mirrors)  # noqa
    scs_internal_repository(name = "scs_internal", mirrors = mirrors)
    sdformat_internal_repository(name = "sdformat_internal", mirrors = mirrors)
    spral_internal_repository(name = "spral_internal", mirrors = mirrors)
    stable_baselines3_internal_repository(name = "stable_baselines3_internal", mirrors = mirrors)  # noqa
    statsjs_internal_repository(name = "statsjs_internal", mirrors = mirrors)
    stduuid_internal_repository(name = "stduuid_internal", mirrors = mirrors)
    styleguide_repository(name = "styleguide_internal", mirrors = mirrors)
    suitesparse_internal_repository(name = "suitesparse_internal", mirrors = mirrors)  # noqa
    sympy_py_internal_repository(name = "sympy_py_internal", mirrors = mirrors)
    tinygltf_internal_repository(name = "tinygltf_internal", mirrors = mirrors)
    tinyobjloader_internal_repository(name = "tinyobjloader_internal", mirrors = mirrors)  # noqa
    tinyxml2_internal_repository(name = "tinyxml2_internal", mirrors = mirrors)
    tomli_internal_repository(name = "tomli_internal", mirrors = mirrors)
    typing_extensions_internal_repository(name = "typing_extensions_internal", mirrors = mirrors)  # noqa
    uritemplate_py_internal_repository(name = "uritemplate_py_internal", mirrors = mirrors)  # noqa
    usockets_internal_repository(name = "usockets_internal", mirrors = mirrors)
    uwebsockets_internal_repository(name = "uwebsockets_internal", mirrors = mirrors)  # noqa
    voxelized_geometry_tools_internal_repository(name = "voxelized_geometry_tools_internal", mirrors = mirrors)  # noqa
    vtk_internal_repository(name = "vtk_internal", mirrors = mirrors)
    xmlrunner_py_internal_repository(name = "xmlrunner_py_internal", mirrors = mirrors)  # noqa
    yaml_cpp_internal_repository(name = "yaml_cpp_internal", mirrors = mirrors)

def _drake_dep_repositories_impl(module_ctx):
    mirrors = DEFAULT_MIRRORS
    blas_repository(name = "blas")
    drake_models_repository(name = "drake_models", mirrors = mirrors)
    gurobi_repository(name = "gurobi")
    lapack_repository(name = "lapack")
    lcm_repository(name = "lcm", mirrors = mirrors)
    meshcat_repository(name = "meshcat", mirrors = mirrors)
    mosek_repository(name = "mosek", mirrors = mirrors)
    opencl_repository(name = "opencl")
    pybind11_repository(name = "pybind11", mirrors = mirrors)
    python_repository(name = "python")
    snopt_repository(name = "snopt")
    x11_repository(name = "x11")
    for name in ["eigen", "fmt", "glib", "spdlog", "zlib"]:
        alias_repository(
            name = name,
            aliases = {name: "@drake//tools/workspace/" + name},
        )

    # Deprecated 2026-01-01.
    buildifier_repository(name = "buildifier", mirrors = mirrors)
    gflags_repository(name = "gflags", mirrors = mirrors)
    gtest_repository(name = "gtest", mirrors = mirrors)
    pycodestyle_repository(name = "pycodestyle", mirrors = mirrors)
    styleguide_repository(name = "styleguide", mirrors = mirrors)

drake_dep_repositories = module_extension(
    implementation = _drake_dep_repositories_impl,
    doc = """(Stable API) Provides access to Drake's dependencies for use by
    downstream projects. See comments in drake/MODULE.bazel for details.""",
)

def _internal_repositories_impl(module_ctx):
    _add_internal_repositories()
    spgrid_module_extension_impl(module_ctx)

internal_repositories = module_extension(
    implementation = _internal_repositories_impl,
    doc = """(Internal use only) Wraps the add_default_repositories repository
    rule into a bzlmod module extension, excluding repositories that are
    already covered by modules, drake_dep_repositories, and crate_universe.""",
)

def _internal_crate_universe_repositories_impl(module_ctx):
    names = crate_universe_repositories_internal(mirrors = DEFAULT_MIRRORS)
    return module_ctx.extension_metadata(
        root_module_direct_deps = names,
        root_module_direct_dev_deps = [],
    )

internal_crate_universe_repositories = module_extension(
    implementation = _internal_crate_universe_repositories_impl,
    doc = """(Internal use only) Wraps the crate_universe repository rules to
    be usable as a bzlmod module extension.""",
)

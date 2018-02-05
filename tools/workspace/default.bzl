# -*- python -*-

load("@drake//tools/workspace/blas:repository.bzl", "blas_repository")
load("@drake//tools/workspace/boost:repository.bzl", "boost_repository")
load("@drake//tools/workspace/buildifier:repository.bzl", "buildifier_repository")  # noqa
load("@drake//tools/workspace/bullet:repository.bzl", "bullet_repository")
load("@drake//tools/workspace/ccd:repository.bzl", "ccd_repository")
load("@drake//tools/workspace/com_google_protobuf:repository.bzl", "com_google_protobuf_repository")  # noqa
load("@drake//tools/workspace/com_jidesoft_jide_oss:repository.bzl", "com_jidesoft_jide_oss_repository")  # noqa
load("@drake//tools/workspace/commons_io:repository.bzl", "commons_io_repository")  # noqa
load("@drake//tools/workspace/drake_visualizer:repository.bzl", "drake_visualizer_repository")  # noqa
load("@drake//tools/workspace/dreal:repository.bzl", "dreal_repository")
load("@drake//tools/workspace/eigen:repository.bzl", "eigen_repository")
load("@drake//tools/workspace/expat:repository.bzl", "expat_repository")
load("@drake//tools/workspace/fcl:repository.bzl", "fcl_repository")
load("@drake//tools/workspace/fmt:repository.bzl", "fmt_repository")
load("@drake//tools/workspace/gflags:repository.bzl", "gflags_repository")
load("@drake//tools/workspace/glew:repository.bzl", "glew_repository")
load("@drake//tools/workspace/glib:repository.bzl", "glib_repository")
load("@drake//tools/workspace/gtest:repository.bzl", "gtest_repository")
load("@drake//tools/workspace/gthread:repository.bzl", "gthread_repository")
load("@drake//tools/workspace/gurobi:repository.bzl", "gurobi_repository")
load("@drake//tools/workspace/ignition_math:repository.bzl", "ignition_math_repository")  # noqa
load("@drake//tools/workspace/ignition_rndf:repository.bzl", "ignition_rndf_repository")  # noqa
load("@drake//tools/workspace/ipopt:repository.bzl", "ipopt_repository")
load("@drake//tools/workspace/lapack:repository.bzl", "lapack_repository")
load("@drake//tools/workspace/lcm:repository.bzl", "lcm_repository")
load("@drake//tools/workspace/lcmtypes_bot2_core:repository.bzl", "lcmtypes_bot2_core_repository")  # noqa
load("@drake//tools/workspace/lcmtypes_robotlocomotion:repository.bzl", "lcmtypes_robotlocomotion_repository")  # noqa
load("@drake//tools/workspace/liblz4:repository.bzl", "liblz4_repository")
load("@drake//tools/workspace/libpng:repository.bzl", "libpng_repository")
load("@drake//tools/workspace/libprotobuf:repository.bzl", "libprotobuf_repository")  # noqa
load("@drake//tools/workspace/mosek:repository.bzl", "mosek_repository")
load("@drake//tools/workspace/net_sf_jchart2d:repository.bzl", "net_sf_jchart2d_repository")  # noqa
load("@drake//tools/workspace/nlopt:repository.bzl", "nlopt_repository")
load("@drake//tools/workspace/numpy:repository.bzl", "numpy_repository")
load("@drake//tools/workspace/octomap:repository.bzl", "octomap_repository")
load("@drake//tools/workspace/optitrack_driver:repository.bzl", "optitrack_driver_repository")  # noqa
load("@drake//tools/workspace/org_apache_xmlgraphics_commons:repository.bzl", "org_apache_xmlgraphics_commons_repository")  # noqa
load("@drake//tools/workspace/protoc:repository.bzl", "protoc_repository")
load("@drake//tools/workspace/pybind11:repository.bzl", "pybind11_repository")
load("@drake//tools/workspace/pycodestyle:repository.bzl", "pycodestyle_repository")  # noqa
load("@drake//tools/workspace/pycps:repository.bzl", "pycps_repository")
load("@drake//tools/workspace/python:repository.bzl", "python_repository")
load("@drake//tools/workspace/scs:repository.bzl", "scs_repository")
load("@drake//tools/workspace/sdformat:repository.bzl", "sdformat_repository")
load("@drake//tools/workspace/semantic_version:repository.bzl", "semantic_version_repository")  # noqa
load("@drake//tools/workspace/snopt:repository.bzl", "snopt_repository")
load("@drake//tools/workspace/spdlog:repository.bzl", "spdlog_repository")
load("@drake//tools/workspace/spruce:repository.bzl", "spruce_repository")
load("@drake//tools/workspace/stx:repository.bzl", "stx_repository")
load("@drake//tools/workspace/styleguide:repository.bzl", "styleguide_repository")  # noqa
load("@drake//tools/workspace/tinydir:repository.bzl", "tinydir_repository")
load("@drake//tools/workspace/tinyobjloader:repository.bzl", "tinyobjloader_repository")  # noqa
load("@drake//tools/workspace/tinyxml2:repository.bzl", "tinyxml2_repository")
load("@drake//tools/workspace/tinyxml:repository.bzl", "tinyxml_repository")
load("@drake//tools/workspace/vtk:repository.bzl", "vtk_repository")
load("@drake//tools/workspace/yaml_cpp:repository.bzl", "yaml_cpp_repository")
load("@drake//tools/workspace/zlib:repository.bzl", "zlib_repository")

def add_default_repositories(excludes = []):
    """Declares workspace repositories for all externals needed by drake (other
    than those built into Bazel, of course).  This is intended to be loaded and
    called from a WORKSPACE file.

    Args:
        excludes: list of string names of repositories to exclude; this can
          be useful if a WORKSPACE file has already supplied its own external
          of a given name.
    """
    if "blas" not in excludes:
        blas_repository(name = "blas")
    if "boost" not in excludes:
        boost_repository(name = "boost")
    if "buildifier" not in excludes:
        buildifier_repository(name = "buildifier")
    if "bullet" not in excludes:
        bullet_repository(name = "bullet")
    if "ccd" not in excludes:
        ccd_repository(name = "ccd")
    if "com_google_protobuf" not in excludes:
        com_google_protobuf_repository(name = "com_google_protobuf")
    if "com_jidesoft_jide_oss" not in excludes:
        com_jidesoft_jide_oss_repository(name = "com_jidesoft_jide_oss")
    if "commons_io" not in excludes:
        commons_io_repository(name = "commons_io")
    if "drake_visualizer" not in excludes:
        drake_visualizer_repository(name = "drake_visualizer")
    if "dreal" not in excludes:
        dreal_repository(name = "dreal")
    if "eigen" not in excludes:
        eigen_repository(name = "eigen")
    if "expat" not in excludes:
        expat_repository(name = "expat")
    if "fcl" not in excludes:
        fcl_repository(name = "fcl")
    if "fmt" not in excludes:
        fmt_repository(name = "fmt")
    if "gflags" not in excludes:
        gflags_repository(name = "gflags")
    if "glew" not in excludes:
        glew_repository(name = "glew")
    if "glib" not in excludes:
        glib_repository(name = "glib")
    if "gtest" not in excludes:
        gtest_repository(name = "gtest")
    if "gthread" not in excludes:
        gthread_repository(name = "gthread")
    if "gurobi" not in excludes:
        gurobi_repository(name = "gurobi")
    if "ignition_math" not in excludes:
        ignition_math_repository(name = "ignition_math")
    if "ignition_rndf" not in excludes:
        ignition_rndf_repository(name = "ignition_rndf")
    if "ipopt" not in excludes:
        ipopt_repository(name = "ipopt")
    if "lapack" not in excludes:
        lapack_repository(name = "lapack")
    if "lcm" not in excludes:
        lcm_repository(name = "lcm")
    if "lcmtypes_bot2_core" not in excludes:
        lcmtypes_bot2_core_repository(name = "lcmtypes_bot2_core")
    if "lcmtypes_robotlocomotion" not in excludes:
        lcmtypes_robotlocomotion_repository(name = "lcmtypes_robotlocomotion")
    if "liblz4" not in excludes:
        liblz4_repository(name = "liblz4")
    if "libpng" not in excludes:
        libpng_repository(name = "libpng")
    if "libprotobuf" not in excludes:
        libprotobuf_repository(name = "libprotobuf")
    if "mosek" not in excludes:
        mosek_repository(name = "mosek")
    if "net_sf_jchart2d" not in excludes:
        net_sf_jchart2d_repository(name = "net_sf_jchart2d")
    if "nlopt" not in excludes:
        nlopt_repository(name = "nlopt")
    if "numpy" not in excludes:
        numpy_repository(name = "numpy")
    if "octomap" not in excludes:
        octomap_repository(name = "octomap")
    if "optitrack_driver" not in excludes:
        optitrack_driver_repository(name = "optitrack_driver")
    if "org_apache_xmlgraphics_commons" not in excludes:
        org_apache_xmlgraphics_commons_repository(name = "org_apache_xmlgraphics_commons")  # noqa
    if "protoc" not in excludes:
        protoc_repository(name = "protoc")
    if "pybind11" not in excludes:
        pybind11_repository(name = "pybind11")
    if "pycodestyle" not in excludes:
        pycodestyle_repository(name = "pycodestyle")
    if "pycps" not in excludes:
        pycps_repository(name = "pycps")
    if "python" not in excludes:
        python_repository(name = "python")
    if "scs" not in excludes:
        scs_repository(name = "scs")
    if "sdformat" not in excludes:
        sdformat_repository(name = "sdformat")
    if "semantic_version" not in excludes:
        semantic_version_repository(name = "semantic_version")
    if "snopt" not in excludes:
        snopt_repository(name = "snopt")
    if "spdlog" not in excludes:
        spdlog_repository(name = "spdlog")
    if "spruce" not in excludes:
        spruce_repository(name = "spruce")
    if "stx" not in excludes:
        stx_repository(name = "stx")
    if "styleguide" not in excludes:
        styleguide_repository(name = "styleguide")
    if "tinydir" not in excludes:
        tinydir_repository(name = "tinydir")
    if "tinyobjloader" not in excludes:
        tinyobjloader_repository(name = "tinyobjloader")
    if "tinyxml2" not in excludes:
        tinyxml2_repository(name = "tinyxml2")
    if "tinyxml" not in excludes:
        tinyxml_repository(name = "tinyxml")
    if "vtk" not in excludes:
        vtk_repository(name = "vtk")
    if "yaml_cpp" not in excludes:
        yaml_cpp_repository(name = "yaml_cpp")
    if "zlib" not in excludes:
        zlib_repository(name = "zlib")

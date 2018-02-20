# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

_REPOSITORY = "RobotLocomotion/pybind11"

_COMMIT = "e19086472c664078b0ba77432001a89f97784c8d"

_SHA256 = "62f1ae603f896e810339a23ed8695e6273d0936c6d77bd150cade67b76b06373"

def pybind11_repository(name):
    github_archive(
        name = name,
        repository = _REPOSITORY,
        commit = _COMMIT,
        sha256 = _SHA256,
        build_file = "@drake//tools/workspace/pybind11:package.BUILD.bazel",
    )

def drake_pybind11_cmake_vars():
    """Provides information for downstream consumption, since projects may be
    sensitive to Pybind versions.
    """
    return {
        "DRAKE_PYBIND11_REPOSITORY": _REPOSITORY,
        "DRAKE_PYBIND11_COMMIT": _COMMIT,
        "DRAKE_PYBIND11_SHA256": _SHA256,
    }

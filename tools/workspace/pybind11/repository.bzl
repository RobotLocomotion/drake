# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pybind11_repository(name):
    github_archive(
        name = name,
        repository = "RobotLocomotion/pybind11",
        commit = "e19086472c664078b0ba77432001a89f97784c8d",
        sha256 = "62f1ae603f896e810339a23ed8695e6273d0936c6d77bd150cade67b76b06373",  # noqa
        build_file = "@drake//tools/workspace/pybind11:package.BUILD.bazel",
    )

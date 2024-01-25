load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "4150a537aca096d8eb2dcde50c5ef10848deafb2",
        sha256 = "c31eac4bebb90e875b5c30f1c35ba8508367d6f650e52730c5623a875918310a",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )

load("//tools/workspace:which.bzl", "which_repository")

def nasm_repository(name):
    which_repository(
        name = name,
        command = "nasm",
    )

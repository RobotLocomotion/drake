# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "d9b1abebba788cb04dbfc208b48de5dc12e742d3",
        sha256 = "d8a0c8b7b4dea2621af5e0f7bdbec47966c9ca5e7b902fa79657bbe6ef763784",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )

def mesh_files():
    """Manual enumeration of mesh files, to avoid needing to write extra Bazel
    logic.

    Recipe to reproduce:
        $ cd models
        $ find ycb/meshes -type f | \
            python -c 'import sys; print(repr(sys.stdin.read().split()))'
    """
    return [
        "ycb/meshes/003_cracker_box_textured.mtl",
        "ycb/meshes/003_cracker_box_textured.obj",
        "ycb/meshes/003_cracker_box_textured.png",
        "ycb/meshes/004_sugar_box_textured.mtl",
        "ycb/meshes/004_sugar_box_textured.obj",
        "ycb/meshes/004_sugar_box_textured.png",
        "ycb/meshes/005_tomato_soup_can_textured.mtl",
        "ycb/meshes/005_tomato_soup_can_textured.obj",
        "ycb/meshes/005_tomato_soup_can_textured.png",
        "ycb/meshes/006_mustard_bottle_textured.mtl",
        "ycb/meshes/006_mustard_bottle_textured.obj",
        "ycb/meshes/006_mustard_bottle_textured.png",
        "ycb/meshes/009_gelatin_box_textured.mtl",
        "ycb/meshes/009_gelatin_box_textured.obj",
        "ycb/meshes/009_gelatin_box_textured.png",
        "ycb/meshes/010_potted_meat_can_textured.mtl",
        "ycb/meshes/010_potted_meat_can_textured.obj",
        "ycb/meshes/010_potted_meat_can_textured.png",
    ]

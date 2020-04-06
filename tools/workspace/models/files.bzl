# -*- python -*-

def ycb_mesh_files():
    """Manual enumeration of mesh files, to avoid needing to write extra Bazel
    logic.

    Recipe to reproduce:
        $ cd models
        $ find ycb/meshes -type f | \
            python3 -c 'import sys; print(repr(sys.stdin.read().split()))'
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
        "ycb/meshes/LICENSE.txt",
    ]

def franka_description_mesh_files():
    return [
        "franka_description/LICENSE",
        "franka_description/meshes/visual/finger.mtl",
        "franka_description/meshes/visual/finger.obj",
        "franka_description/meshes/visual/hand.mtl",
        "franka_description/meshes/visual/hand.obj",
        "franka_description/meshes/visual/link0.mtl",
        "franka_description/meshes/visual/link0.obj",
        "franka_description/meshes/visual/link1.mtl",
        "franka_description/meshes/visual/link1.obj",
        "franka_description/meshes/visual/link2.mtl",
        "franka_description/meshes/visual/link2.obj",
        "franka_description/meshes/visual/link3.mtl",
        "franka_description/meshes/visual/link3.obj",
        "franka_description/meshes/visual/link4.mtl",
        "franka_description/meshes/visual/link4.obj",
        "franka_description/meshes/visual/link5.mtl",
        "franka_description/meshes/visual/link5.obj",
        "franka_description/meshes/visual/link6.mtl",
        "franka_description/meshes/visual/link6.obj",
        "franka_description/meshes/visual/link7.mtl",
        "franka_description/meshes/visual/link7.obj",
    ]

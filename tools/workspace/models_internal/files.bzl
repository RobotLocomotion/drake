# -*- python -*-

# Keep the macros sorted alphabetically by macro name.
# Keep the lists of files sorted alphabetically by filename.

def dishes_files():
    return [
        "dishes/bowls/evo_bowl_no_mtl.obj",
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

def jaco_description_mesh_files():
    return [
        "jaco_description/LICENSE",
        "jaco_description/meshes/arm.obj",
        "jaco_description/meshes/arm_half_1.obj",
        "jaco_description/meshes/arm_half_2.obj",
        "jaco_description/meshes/arm_mico.obj",
        "jaco_description/meshes/base.obj",
        "jaco_description/meshes/finger_distal.obj",
        "jaco_description/meshes/finger_proximal.obj",
        "jaco_description/meshes/forearm.obj",
        "jaco_description/meshes/forearm_mico.obj",
        "jaco_description/meshes/hand_2finger.obj",
        "jaco_description/meshes/hand_3finger.obj",
        "jaco_description/meshes/ring_big.obj",
        "jaco_description/meshes/ring_small.obj",
        "jaco_description/meshes/shoulder.obj",
        "jaco_description/meshes/wrist.obj",
        "jaco_description/meshes/wrist_spherical_1.obj",
        "jaco_description/meshes/wrist_spherical_2.obj",
    ]

def skydio_2_mesh_files():
    return [
        "skydio_2/skydio_2_1000_poly.mtl",
        "skydio_2/skydio_2_1000_poly.obj",
        "skydio_2/skydio_2.png",
        "skydio_2/LICENSE",
    ]

def tri_homecart_mesh_files():
    return [
        "tri_homecart/LICENSE.txt",
        "tri_homecart/homecart_arm_mount_cantilever.mtl",
        "tri_homecart/homecart_arm_mount_cantilever.obj",
        "tri_homecart/homecart_arm_mount_stack.mtl",
        "tri_homecart/homecart_arm_mount_stack.obj",
        "tri_homecart/homecart_basecart.mtl",
        "tri_homecart/homecart_basecart.obj",
        "tri_homecart/homecart_basecart_wood_color.png",
        "tri_homecart/homecart_baseplate.mtl",
        "tri_homecart/homecart_baseplate.obj",
        "tri_homecart/homecart_bimanual_upper_structure.mtl",
        "tri_homecart/homecart_bimanual_upper_structure.obj",
        "tri_homecart/homecart_cutting_board.mtl",
        "tri_homecart/homecart_cutting_board.obj",
        "tri_homecart/homecart_cutting_board_color.png",
    ]

def ur3e_mesh_files():
    return [
        "ur3e/LICENSE.TXT",
        "ur3e/base.mtl",
        "ur3e/base.obj",
        "ur3e/forearm.mtl",
        "ur3e/forearm.obj",
        "ur3e/shoulder.mtl",
        "ur3e/shoulder.obj",
        "ur3e/upperarm.mtl",
        "ur3e/upperarm.obj",
        "ur3e/ur3e_color.png",
        "ur3e/ur3e_normal.png",
        "ur3e/ur3e_occlusion_roughness_metallic.png",
        "ur3e/wrist1.mtl",
        "ur3e/wrist1.obj",
        "ur3e/wrist2.mtl",
        "ur3e/wrist2.obj",
        "ur3e/wrist3.mtl",
        "ur3e/wrist3.obj",
    ]

def wsg_50_description_mesh_files():
    return [
        "wsg_50_description/meshes/finger_without_tip.obj",
        "wsg_50_description/meshes/finger_with_tip.obj",
        "wsg_50_description/meshes/wsg_body.obj",
    ]

def wsg_50_hydro_bubble_mesh_files():
    return [
        "wsg_50_hydro_bubble/meshes/bubble_finger.mtl",
        "wsg_50_hydro_bubble/meshes/bubble_finger.obj",
        "wsg_50_hydro_bubble/meshes/ellipsoid_bubble_geometry.mtl",
        "wsg_50_hydro_bubble/meshes/ellipsoid_bubble_geometry.obj",
    ]

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

from collections.abc import Sequence
import numpy as np
import os
import re


from pydrake.geometry import (
    ReadObjToTriangleSurfaceMesh,
)
from pydrake.multibody.tree import (
    CalcSpatialInertia,
    SpatialInertia_,
)

_PACKAGE_TEMPLATE = """
<?xml version="1.0"?>
<package format="2">
  <name>{name}</name>
  <version>0.0.0</version>
  <description>
    Package generated by mesh_to_model from mesh file: {mesh_name}.
  </description>
  <maintainer email="nobody@example.com">Nobody</maintainer>
  <license>unknown</license>
</package>
"""
_VISUAL_TEMPLATE = """
      <visual name='visual'>
        <geometry>
          <pose>{geometry_position} 0 0 0</pose>
          <mesh>
            <uri>{mesh_uri}</uri>
            <scale>{mesh_scale}</scale>
          </mesh>
        </geometry>
      </visual>"""
_COLLISION_TEMPLATE = """
      <collision name='collision'>
        <geometry>
          <pose>{geometry_position} 0 0 0</pose>
          <mesh>
            <uri>{mesh_uri}</uri>
            <scale>{mesh_scale}</scale>
          </mesh>
        </geometry>
      </visual>"""
_SDF_TEMPLATE = """<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='{name}'>
    <link name='{name}'>
      <inertial>
        <pose>{inertial_pose}</pose>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx>
          <ixy>{ixy}</ixy>
          <ixz>{ixz}</ixz>
          <iyy>{iyy}</iyy>
          <iyz>{iyz}</iyz>
          <izz>{izz}</izz>
        </inertia>
      </inertial>{visual}{collision}
    </link>
  </model>
</sdf>
"""


class MeshMassSpec:
    """Controls how the mass is to be computed: density or absolute mass."""
    def __init__(self, density: float = None, mass: float = None):
        """
        Defines how the mass is specified -- via density *or* mass. At least
        one value must be defined. If mass is provided, it will be used.
        """
        if density is None and mass is None:
            raise ValueError("Neither mass nor density were defined.")
        self.density = density
        self.mass = mass


class MeshFramePose:
    """
    Controls the relative positions of geometry and body frames.

    There are three options:

        - Body origin at geometry origin.
        - Body origin at geometry center of mass.
        - Body origin at arbitrary point (measured and expressed in geometry
          frame).
    """
    def __init__(self, at_com: bool = False,
                 p_GoBo: Sequence[tuple[float, float, float]] = None):
        if at_com and p_GoBo is not None:
            raise ValueError(
                "Specify either at_com = True or p_GoBo, not both.")
        self.at_com = at_com
        self.p_GoBo = None if p_GoBo is None else np.array(p_GoBo)


def _read_package_name(package_path: str):
    """
    Reads the ROS package manifest file at the given path, returning the value
    of the name tag.
    """
    with open(package_path) as f:
        text = "".join(f.readlines())
    match = re.search("<name>(.+?)</name>", text)
    return match.group(1)


def _resolve_package_prefix(package_spec: str, mesh_path: str):
    """
    Given an encoded package protocol, provides the file prefix to be used in
    referencing the mesh file.

    Args:
        package_spec: Can be one of "none", "auto", or a path to a package.xml
            file.
        mesh_path: The path to the mesh file being processed. Used if
            package_spec is "auto". A package.xml file in that directory will
            be used or created.

    Returns:
        A string that can be prefixed to the mesh file name to be used as the
        mesh URI.
    """
    if package_spec == "none":
        return ""

    package_name = None
    if package_spec.endswith("package.xml"):
        package_name = _read_package_name(package_spec)

    if package_spec == "auto":
        mesh_dir = os.path.dirname(os.path.abspath(mesh_path))
        candidate_package_path = os.path.join(mesh_dir, "package.xml")
        if os.path.exists(candidate_package_path):
            package_name = _read_package_name(candidate_package_path)
        else:
            # The name of the *package* is the name of the mesh file's
            # directory.
            package_name = os.path.split(mesh_dir)[-1]
            subs = {"name": package_name,
                    "mesh_name": mesh_path.split()[-1]}
            with open(candidate_package_path, "w") as f:
                f.write(_PACKAGE_TEMPLATE.format(**subs))

    if package_name is not None:
        return f"package://{package_name}/"

    raise ValueError(f"Unrecognized package specification: '{package_spec}'.")


def mesh_to_model(mesh_path: str, scale: float, model_name: str,
                  mass_spec: MeshMassSpec, frame_pose: MeshFramePose,
                  no_collision: bool, no_visual: bool,
                  encoded_package: str):
    """
    Creates an SDF file based on the geometry defined in a mesh file.
    """
    mesh_G = ReadObjToTriangleSurfaceMesh(filename=mesh_path, scale=scale)

    # TODO(SeanCurtis-TRI): Confirm that the mesh is watertight.

    if mass_spec.mass is not None:
        # If mass is defined, it wins.
        # Neither the unit inertia nor the center of mass depend on the
        # density. So, we can compute those quantities for an arbitrary
        # density and then reconfigure the SpatialInertia with the desired
        # mass.
        M_GGo_G = CalcSpatialInertia(mesh=mesh_G, density=1.0)
        M_GGo_G = SpatialInertia_[float](mass=mass_spec.mass,
                                         p_PScm_E=M_GGo_G.get_com(),
                                         G_SP_E=M_GGo_G.get_unit_inertia())
    else:
        M_GGo_G = CalcSpatialInertia(mesh=mesh_G, density=mass_spec.density)

    p_GoGcm = M_GGo_G.get_com()
    M_GGcm_G = M_GGo_G.Shift(p_GoGcm)
    mass = M_GGo_G.get_mass()

    if frame_pose.p_GoBo is None:
        if frame_pose.at_com:
            p_BoGo = -p_GoGcm
            p_BoBcm = (0.0, 0.0, 0.0)
        else:
            # Default case.
            p_BoGo = (0.0, 0.0, 0.0)
            p_BoBcm = p_GoGcm
    else:
        p_BoGo = -frame_pose.p_GoBo
        p_BoBcm = p_GoGcm - frame_pose.p_GoBo

    # In SDF files, the inertia tensor is always reported around the center
    # of mass.
    I_GGcm_G = M_GGcm_G.CalcRotationalInertia()
    moments = I_GGcm_G.get_moments()
    products = I_GGcm_G.get_products()

    # Figure out the mesh URI and model names.
    package_prefix = _resolve_package_prefix(encoded_package, mesh_path)
    mesh_file_name = os.path.basename(mesh_path)
    mesh_uri = package_prefix + mesh_file_name

    # Possible model name.
    mesh_stem = os.path.splitext(mesh_file_name)[0]

    # TODO(SeanCurtis-TRI) Do we care about the nature of the precision being
    # printed on positions and inertia values? Currently, everything is just
    # being hit with {:g}.

    subs = dict()
    # model_name is None *or* model_name == "" prefers mesh_stem.
    subs["name"] = model_name or mesh_stem
    # With zero rotation, this is simply the position of com in B.
    p_BoBcm_str = " ".join([f"{x:g}" for x in p_BoBcm])
    subs["inertial_pose"] = f"{p_BoBcm_str} 0 0 0"
    subs["mass"] = mass
    subs["ixx"] = f"{moments[0]:g}"
    subs["iyy"] = f"{moments[1]:g}"
    subs["izz"] = f"{moments[2]:g}"
    subs["ixy"] = f"{products[0]:g}"
    subs["ixz"] = f"{products[1]:g}"
    subs["iyz"] = f"{products[2]:g}"
    subs["mesh_uri"] = mesh_uri
    subs["mesh_scale"] = "{s:g} {s:g} {s:g}".format(s=scale)
    subs["geometry_position"] = " ".join([f"{x:g}" for x in p_BoGo])
    subs["visual"] = "" if no_visual else _VISUAL_TEMPLATE.format(**subs)
    # TODO(SeanCurtis-TRI): Determine if we can apply <drake:declare_convex/>
    # based on mesh analysis.
    subs["collision"] = \
        "" if no_collision else _COLLISION_TEMPLATE.format(**subs)

    return _SDF_TEMPLATE.format(**subs)

"""
Utility for converting a mesh file into an SDFormat file containing a single
model.

Currently only meshes in Wavefront OBJ files are supported. Other geometry
formats will be supported in the future.

The model contains a single body. The model and the body share the same name.
The mass properties of the body are computed based on the enclosed volume of
the mesh (make sure the mesh is actually a closed manifold). Finally, the mesh
is (optionally) registered as visual and collision geometries for the body.

Usage:

Model scale::

Drake uses meters as distance of length. Modelling packages may frequently use
other units (e.g. centimeters). This program will _assume_ the units are
meters. If they are not, provide the scale conversion factor to go from mesh
units to meters.

Configuring mass::

The total mass of the body can be configured through one of two ways:
specifying the density of a homogeneous material which completely fills the
enclosed volume of the mesh, or specifying the total mass of the body. If
neither is specified, it is assumed to have the density of water (~1000kg/m³).
Only one of density and mass can be specified.

Geometry frame vs Body Frame::

The mesh's vertices are measured and expressed in an arbitrary geometry frame
G. By default, the body frame B is coincident with G (i.e., X_BG = I). The
two frames will always be aligned (R_BG = R_GB = I), but the relative positions
of the origins (p_GoBo) can be modified. You can either request that the
center of mass be coincident with the computed center of mass or specify an
arbitrary value for p_GoBo (the body's origin position relative to the mesh).

Collision and visualization::

By default, the mesh is used for both collision and visual geometric roles.
Either of these can be suppressed.

Packages::

If the mesh is used in a <visual> or <collision> element (see above) the mesh
must be referenced via some url. The default mesh url is a simple relative path
assuming the SDFormat file and mesh files are in the same directory. There are
two alternatives: create/use a package at the mesh file location or specify a
particular package.

    Default behavior::

        When calling

            mesh_to_model path/to/my_models/mesh.obj

        the resulting visual element will look like:

            <geometry><mesh><uri>mesh.obj</uri></mesh></geometry>

        This uri resolution is not SDFormat standard but is acceptable as a
        Drake parser extension.

    Specify package::

        When calling

            mesh_to_model --package path/to/my_package/package.xml \
                  path/to/my_models/mesh.obj

        the resulting visual element will look like:

            <geometry><mesh><uri>package://my_package/mesh.obj</uri></mesh></geometry>

        This example assumes that the <name> field in the indicated package.xml
        is also "my_package". Errors in reading the file (or the package name
        from the manifest) will cause the program to fail.

    Create/use package at mesh file location::

        When calling

            mesh_to_model --package auto path/to/my_models/mesh.obj

        the program will look for the existence of
        path/to/my_models/package.xml. If it doesn't exist, it will be created.
        The visual element will look like:

            <geometry><mesh><uri>package://my_models/mesh.obj</uri></mesh></geometry>

        The package.xml file will minimally compliant but may contain
        meanignless values (see
        http://wiki.ros.org/Manifest#catkin.2Fpackage.xml.Required_Tags). Any
        error in reading or writing the file will cause the program to fail.

Running::

    From a Drake source build, run this as::

        bazel run //tools:mesh_to_model -- --help

    From a Drake binary release (including pip releases), run this as::

        python3 -m pydrake.multibody.mesh_to_model path/to/mesh.obj
"""

import argparse
import logging
import numpy as np

from pydrake.common import configure_logging as _configure_logging
from pydrake.multibody._mesh_to_model import (
    MeshModelMaker as _MeshModelMaker,
    MeshMassSpec as _MeshMassSpec,
    MeshFramePose as _MeshFramePose,
)

_logger = logging.getLogger("drake")


class _MyFormatter(argparse.ArgumentDefaultsHelpFormatter,
                   argparse.RawDescriptionHelpFormatter):
    """
    A formatter that leaves the description formatted as above *and* prints the
    default values for variables.
    """
    pass


def _main():
    _configure_logging()
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=_MyFormatter)
    parser.add_argument(
        "--model-name", action="store", default=None,
        help=("The optional name to assign to both model and body. If None "
              "the mesh file name is used."))
    parser.add_argument(
        "--scale", action="store", default=1.0, type=float,
        help="The scale for scaling the mesh vertex positions to meters.")

    mass_group = parser.add_mutually_exclusive_group()
    mass_group.add_argument(
        "--density", action="store", default=1000, type=float,
        help=("The density of the object in kg/m³. Only specify one of "
              "--density and --mass. If neither is specified, the default "
              "density of water is used."))
    mass_group.add_argument(
        "--mass", action="store", default=None, type=float,
        help=("The mass of the object in kg. Only specify one of --density "
              "and --mass. If neither is specified, default density is used."))

    com_group = parser.add_mutually_exclusive_group()
    com_group.add_argument(
        "--origin-at-com", action="store_true", default=False,
        help=("If requested, the body's origin defined at the geometry's "
              "center of mass. Only specify one of --body-origin and "
              "--origin-at-com."))
    # TODO(SeanCurtis-TRI) It would be nice if the help displayed as:
    #     --body-origin COM_X COM_Y COM_Z
    # instead of
    #     --body-origin COM COM COM
    com_group.add_argument(
        "--body-origin", nargs=3, type=float, action="store",
        default=None, metavar="COM",
        help=("Specify the body origin in the mesh's canonical frame. Only "
              "specify one of --body-origin and --origin-at-com."))

    parser.add_argument(
        "--package", type=str, action="store", default="none",
        help=("Specify the package semantics used in the resulting SDFormat "
              "file. Options are: 'none' (the mesh will be referenced using a "
              "relative path in the same directory), 'auto' (current "
              "directory is treated as a package root and package.xml will be "
              "added if not already present), or path to a package.xml file "
              "(the mesh will be referenced as being in the root of that "
              "package)."))

    parser.add_argument(
        "--no-collision", action="store_true", default=False,
        help=("If specified, the mesh will *not* be used as collision "
              "geometry."))
    parser.add_argument(
        "--no-visual", action="store_true", default=False,
        help="If specified, the mesh will *not* be used as visual geometry.")

    parser.add_argument(
        "mesh", default=None,
        help="The path to the mesh file to process.")

    parser.add_argument(
        "output", default=None,
        help="The path where the SDFormat model will be written.")

    args = parser.parse_args()

    maker = _MeshModelMaker()
    maker.model_name = args.model_name
    maker.scale = args.scale
    maker.mass_spec = _MeshMassSpec(mass=args.mass, density=args.density)
    maker.frame_pose = _MeshFramePose(at_com=args.origin_at_com,
                                      p_GoBo=args.body_origin)
    maker.no_collision = args.no_collision
    maker.no_visual = args.no_visual
    maker.encoded_package = args.package
    maker.make_model(args.mesh, args.output)


if __name__ == "__main__":
    _main()

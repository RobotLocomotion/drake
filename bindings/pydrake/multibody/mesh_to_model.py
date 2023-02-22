r"""
Utility for converting a mesh file into an SDFormat file containing a single
model.

Currently only meshes in Wavefront OBJ files are supported. Other geometry
formats will be supported in the future.

Preconditions on the OBJ:

  - The mesh is "watertight" (no cracks, no openings).
  - The mesh is closed (i.e., no sheets of triangles).

Properties of the resulting SDFormat file:

  - The model contains a single body.
  - The body and model will share a common name.
  - The mass properties are computed based on the volume enclosed by the mesh.
    (If the preconditions are not met, mass properties will nevertheless be
    computed, but they are not guaranteed to be correct).
  - The mesh will be used as both <collision> and <visual> geometries.

**Running**:

    From a Drake source build, run this as::

            bazel run //tools:mesh_to_model -- --help

    From a Drake binary release (including pip releases), run this as::

            python3 -m pydrake.multibody.mesh_to_model path/to/mesh.obj

**Model scale**:

    Drake uses meters as the unit of length. Modelling packages may frequently
    use other units (e.g. centimeters). This program will *assume* the units
    are meters. If they are not, provide the scale conversion factor to go from
    mesh units to meters. For example, if your mesh is in centimeters, use 0.01
    for the scale.

**Configuring mass**:

    The body's mass can be configured through one of two ways: specifying the
    _density_ of a homogeneous material which completely fills the enclosed
    volume of the mesh, or specifying the total _mass_ of the body. If neither
    is specified, it is assumed to have the density of water (~1000kg/m³). Only
    one of density and mass can be specified.

**Geometry frame vs body frame**:

    The mesh's vertices are measured and expressed in an arbitrary geometry
    frame G. By default, the body frame B is coincident with G (i.e.,
    X_BG = I). The two frames will always be aligned (R_BG = R_GB = I), but the
    relative positions of the origins (p_GoBo) can be modified. You can either
    request that the body origin be coincident with the computed center of mass
    or specify an arbitrary value for p_GoBo (the body's origin position
    relative to the mesh's origin).

**Packages**:

    If the mesh is used in a <visual> or <collision> element (see above) the
    mesh must be referenced via some url. The default mesh url is a simple
    relative path assuming the SDFormat file and mesh files are in the same
    directory. There are two alternatives: create/use a package at the mesh
    file location or specify a package.

        Default behavior:

            When calling::

                mesh_to_model path/to/my_models/mesh.obj

            the resulting mesh's <uri> element will look like::

                <uri>mesh.obj</uri>

            This uri resolution is not SDFormat standard but is acceptable as a
            Drake parser extension. It is your responsibility to place the mesh
            in the same folder as the sdf.

        Specify package:

            When calling::

                mesh_to_model --package=path/to/package.xml \
                    path/to/my_models/mesh.obj

            the resulting mesh's <uri> element will look like::

                <uri>package://my_package/my_models/mesh.obj</uri>

            This example assumes that the <name> field in the indicated
            package.xml is "my_package". Errors in reading the file (or the
            package name from the manifest) will cause the program to fail. The
            mesh file must live somewhere underneath the package.xml directory.

        Create/use package at mesh file location:

            When calling::

                mesh_to_model --package=auto path/to/my_models/mesh.obj

            the program will look for the existence of
            path/to/my_models/package.xml. If it doesn't exist, it will be
            created. The resulting mesh's <uri> element will look like::

                <uri>package://my_models/mesh.obj</uri>

            The package.xml file will be minimally compliant but may contain
            meaningless values (see
            http://wiki.ros.org/Manifest#catkin.2Fpackage.xml.Required_Tags).
            Any error in reading or writing the file will cause the program to
            fail.
"""

import argparse
import logging
import numpy as np
import os
from pathlib import Path

from pydrake.common import configure_logging as _configure_logging
from pydrake.multibody._mesh_model_maker import (
    MeshModelMaker as _MeshModelMaker,
)

_logger = logging.getLogger("drake")


def _CommaSeparatedXYZ(arg: str):
    """An argparse parser for an x,y,z vector."""
    x, y, z = [float(item) for item in arg.split(',')]
    return np.array([x, y, z])


# In the event that the user passes in a malformed value for --body-origin,
# this makes the error message nicer.
_CommaSeparatedXYZ.__name__ = "comma-separated triple"


def _main():
    _configure_logging()

    default_maker = _MeshModelMaker(mesh_path=None, output_dir=None)

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--model-name", metavar="NAME",
        help=("The optional name to assign to both model and body. If none "
              "is given, the mesh file name is used."))
    parser.add_argument(
        "--scale", default=default_maker.scale, type=float,
        metavar="FACTOR",
        help=("The scale for scaling the mesh vertex positions to meters. "
              "Must be positive."))

    mass_group = parser.add_mutually_exclusive_group()
    mass_group.add_argument(
        "--density", default=default_maker.density,
        type=float, metavar="KG/M3",
        help=("The density of the object in kg/m³. Only specify one of "
              "--density and --mass. If neither is specified, the default "
              "density of water is used."))
    mass_group.add_argument(
        "--mass", type=float, metavar="KG",
        help=("The mass of the object in kg. Only specify one of --density "
              "and --mass. If neither is specified, default density of water "
              "is used."))

    com_group = parser.add_mutually_exclusive_group()
    com_group.add_argument(
        "--origin-at-com", action="store_true", dest="at_com",
        help=("If requested, the body's origin is defined at the geometry's "
              "center of mass. Only specify one of --body-origin and "
              "--origin-at-com."))
    com_group.add_argument(
        "--body-origin", type=_CommaSeparatedXYZ, dest="p_GoBo",
        metavar="X,Y,Z",
        help=("Specify the body origin in the mesh's canonical frame "
              "(p_GoBo_G). Only specify one of --body-origin and "
              "--origin-at-com."))

    parser.add_argument(
        "--package", type=str, dest="encoded_package",
        default=default_maker.encoded_package,
        metavar="ENCODING",
        help=("Specify the package semantics used in the resulting SDFormat "
              "file. Options are: 'none' (default): the mesh will be "
              "referenced using a relative path in the same directory, "
              "'auto': current directory is treated as a package root and "
              "package.xml will be added if not already present), or "
              "[path to a package.xml file]: the mesh will be referenced as "
              "being in the root of that package)."))

    parser.add_argument(
        "--output-dir", type=Path,
        help=("The path where the SDFormat model will be written. If not "
              "given, the file will be written in the mesh's directory. If "
              "given, the directory must already exist. The model file will "
              "share the same stem name as the mesh file."))

    parser.add_argument(
        "mesh_path", type=Path,
        help="The path to the mesh file to process.")

    args = parser.parse_args()

    if 'BUILD_WORKSPACE_DIRECTORY' in os.environ:
        os.chdir(os.environ['BUILD_WORKING_DIRECTORY'])

    maker = _MeshModelMaker(**vars(args))
    maker.make_model()


if __name__ == "__main__":
    _main()

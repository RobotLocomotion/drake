"""Given the output of Meshlab, suggests an SDF fragment for a mesh.

This partially automates the instructions from:
http://gazebosim.org/tutorials?tut=inertia

(1) Open Meshlab for the OBJ in question.

In Meshlab, select the menu option:
  Filters ->
    Quality Measure and Computations ->
      Compute Geometric Measures

(2) Copy + paste the entire output of Meshlab (from your console window) into a
text file, e.g., myobject_meshlab_log.txt.

(3) Run this tool, passing the text file name as an argument, along with the
mass and scale of your model.

If your inertia matrix values are too small, rescale the mesh in Meshlab (but
don't save it!) prior to computing the Geometric Measures.  Be sure that
the --scale value passed to this tool reflects the rescaling.

This tool has been tested only with Meshlab v1.3.2, which is the version
available in Ubuntu Xenial 16.04.  It might not work with other versions.
"""


import argparse
import os
import re
import sys

_TEMPLATE = """<?xml version='1.0'?>
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
      </inertial>
      <visual name='visual'>
        <geometry>
          <mesh>
            <uri>{mesh_uri}</uri>
            <scale>{mesh_scale}</scale>
          </mesh>
        </geometry>
    </link>
  </model>
</sdf>
"""


def _search(log_text, regexp):
    result = re.search(regexp, log_text)
    if result is None:
        raise RuntimeError("Could not find '{}' in '{}'".format(
            regexp, log_text))
    return result.groups()


def _rescale(value_string, scale):
    result = float(value_string) * scale
    if result == 0.0:
        # Remove negative zeros.
        result = 0.0
    return result


def convert(log_text, scale, mass_kg):
    # Match "LOG: 0 Opened mesh myfile.obj in 54 msec".
    obj_filename, = _search(log_text, "Opened mesh (.*) in .* msec")
    model_name, _ = os.path.splitext(os.path.basename(obj_filename))

    # Match "LOG: 2 Center of Mass  is 0.421246 -0.000000 4.383873".
    log_text_com, _ = _search(
        log_text,
        "Center of Mass  is(( [-.0-9]+){3})\n")
    com_x, com_y, com_z = map(
        lambda token: _rescale(token, scale),
        log_text_com.split())

    # Match: "LOG: 2 Mesh Volume  is 147.826782".
    log_text_volume, = _search(
        log_text,
        "Mesh Volume  is ([-.0-9]+)\n")
    volume_m3 = _rescale(log_text_volume, scale ** 3)

    # Match:
    # LOG: 2 Inertia Tensor is :
    # LOG: 2     | 2053.508545  -0.000000  -72.085564 |
    # LOG: 2     | -0.000000  2370.329590  -0.000000 |
    # LOG: 2     | -72.085564  -0.000000  1909.948364 |
    elements = _search(
        log_text,
        r"LOG. 2 +Inertia Tensor is .\n"
        r"LOG. 2 +\| +([-.0-9]+) +([-.0-9]+) +([-.0-9]+) +\|\n"
        r"LOG. 2 +\| +([-.0-9]+) +([-.0-9]+) +([-.0-9]+) +\|\n"
        r"LOG. 2 +\| +([-.0-9]+) +([-.0-9]+) +([-.0-9]+) +\|\n"
    )
    inertia_scale = (scale ** 5) * mass_kg / volume_m3
    ixx = _rescale(elements[0], inertia_scale)
    ixy = _rescale(elements[1], inertia_scale)
    ixz = _rescale(elements[2], inertia_scale)
    iyx = _rescale(elements[3], inertia_scale)
    iyy = _rescale(elements[4], inertia_scale)
    iyz = _rescale(elements[5], inertia_scale)
    izx = _rescale(elements[6], inertia_scale)
    izy = _rescale(elements[7], inertia_scale)
    izz = _rescale(elements[8], inertia_scale)
    assert ixy == iyx
    assert ixz == izx
    assert iyz == izy

    # Reject too-small inertia matrices.  Meshlab truncates its printouts to
    # 0.000001; if the inertia values are too small, they may round to zero or
    # have too little precision.
    if max([float(e) for e in elements]) < 1e-3:
        raise RuntimeError(
            "The inertia matrix elements are too small; "
            "rescale the mesh to have more volume, then try again")

    # We want to print the inertia matrix consistently.  So, given a precision
    # (as number of significant figures) for the floating point General Format
    # in https://docs.python.org/3.4/library/string.html, we'll determine the
    # Fixed Point precision of the maximum element that yields the correct
    # number of sig figs.  Then we'll apply that to all of the other elements.
    inertia_sigfigs = 3
    max_element = max([ixx, ixy, ixz, iyy, iyz, izz])
    exp = int("{:.{}e}".format(max_element, inertia_sigfigs).split('e')[-1])
    inertia_precision = max(0, inertia_sigfigs - 1 - exp)
    inertia_format_template = "{: .#f}".replace('#', str(inertia_precision))

    subs = dict()
    subs["name"] = model_name
    subs["inertial_pose"] = "{x:.3g} {y:.3g} {z:.3g} 0 0 0".format(
        x=com_x, y=com_y, z=com_z)
    subs["mass"] = "{:.3g}".format(mass_kg)
    subs["ixx"] = inertia_format_template.format(ixx)
    subs["ixy"] = inertia_format_template.format(ixy)
    subs["ixz"] = inertia_format_template.format(ixz)
    subs["iyy"] = inertia_format_template.format(iyy)
    subs["iyz"] = inertia_format_template.format(iyz)
    subs["izz"] = inertia_format_template.format(izz)
    subs["mesh_uri"] = obj_filename
    subs["mesh_scale"] = "{s:g} {s:g} {s:g}".format(
        s=scale)
    return _TEMPLATE.format(**subs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scale", type=float, required=True,
        help="Mesh scale factor that yields meters; "
        "e.g., a mesh drawn in centimeters should supply 0.01."
        "If you have rescaled the mesh for numerical reasons, you must "
        "incorporate both the original units and rescaling here so that "
        "the result is still meters."
    )
    parser.add_argument(
        "--mass_kg", type=float, required=True,
        help="Mass of the object")
    parser.add_argument(
        "filename", nargs=1,
        help="A foo.txt file that contains meshlab log output.")
    args = parser.parse_args()

    filename = args.filename[0]
    _, ext = os.path.splitext(os.path.basename(filename))
    if ext == ".txt":
        with open(filename) as log_file:
            log_text = "".join(log_file.readlines())
        sdf_text = convert(log_text, args.scale, args.mass_kg)
        print(sdf_text)
    else:
        print("Can't handle a '{}' file (wanted '.txt')".format(
            ext))
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())

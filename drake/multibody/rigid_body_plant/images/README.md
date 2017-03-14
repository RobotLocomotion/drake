# Editing and Creating images

## SVG files

The simplest way to create a .png from the .svg file is to use Inkscape.

To install: `sudo apt-get install inkscape`

The version of inkscape will depend on which version of ubuntu you are running.
There are differences between Trusty (14.04) and Xenial (16.04).  These
differences manifest themselves in some of the relevant GUI.  In those cases,
the two options will be shown as [Trusty|Xenial].

The following steps will re-create the png:

1. Launch inkscape
2. Open the .svg file.  File -> Open
3. Export a png
   1. File -> [Export Bitmap...|Export PNG Image...]
   2. In "Export area", select "Drawing"
   3. In "Bitmap size", set "pixels at" to 100.0 dpi
   4. Click [Browse|Export As] and direct it to the
   `$DRAKE_DISTRO$/drake/multibody/rigid_body_plant/images` directory.
   5. Use the same file name as the .svg file (e.g., image.svg --> image.png).
   6. Click the "Export" button.

## Python files

These python scripts use Python 2.X, matplotlib, and numpy to generate the
 correspondingly named .png files.

Acquiring the dependencies in Ubuntu:

- `sudo apt-get install python-matplotlib`
- `sudo apt-get install python-numpy`

To create the image:

1. `cd $DRAKE_DISTRO$/drake/multibody/rigid_body_plant/images`
2. `python file_name.py`

This will create the image `file_name.png`.

# MultibodyPlant Images

## SVG files to PNG Files

The simplest way to create a `.png` from the `.svg` file is to use Inkscape.

First, ensure it is installed: `sudo apt install inkscape`. Then:

```sh
cd drake/multibody/plant/images
inkscape --without-gui --export-area-drawing \
    simple_contact.svg -e simple_contact.png
```

## Python-Generated PNG Files

To create the images:

```sh
cd drake/multibody/plant/images
python3 ideal_stiction.py
python3 stiction.py
```

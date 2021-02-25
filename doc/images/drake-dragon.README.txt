This SVG does not render well in Firefox (nor in Inkscape), so we rasterize it.

Steps used to generate the accompanying PNG:

```sh
# For Ubuntu 18.04.
sudo apt install inkscape imagemagick-6.q16

cd drake/doc/images
# Export with inkscape.
inkscape --without-gui --export-area-drawing --export-dpi=300 \
    drake-dragon.svg -e drake-dragon.png
# Scale down with ImageMagick.
convert ./drake-dragon.png -resize 25% -strip ./drake-dragon.png

# Generate favicon.
convert ./drake-dragon.png \
    -resize 200x200 -gravity center -background none -extent 200x200 -strip \
    favicon.png
```

*Note*:
I (Eric) tried 96 and 150 DPI. Did not look great.

I also tried stuff like
```sh
convert -units PixelsPerInch ./drake-dragon.png \
    -density 300 -resize 800x800 drake-dragon.png
```
but it looked bad.

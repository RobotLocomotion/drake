This directory contains files created for the YCB object model database.
To use this code within Drake and drake-visualizer, you must download
additional model files from a YCB database, then add some of those files to the
Drake model directory. We'll use the YCB mug (#25) as an example.

There are several variants of YCB data. We recommend these scans at CMU:
https://cmu.app.box.com/s/q436qspqfmahxzpv4qtpf99li71fqes3

Using the YCB mug as an example, download and unzip 025_mug.tar.gz.

From your download, copy these files into the Drake Ycb025Mug directory: 
copy 025_mug/google_16k/textured.obj to Ycb025Mug.obj
copy 025_mug/google_16k/texture_map.png to texture_map.png

Now the Ycb025Mug model is ready to use within Drake and drake-visualizer.

Note on Ycb025Mug specifically: for the mesh Ycb025Mug.obj, the bottom inside 
of the mug is not flat, and does not match the real mug.

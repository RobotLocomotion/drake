#!/bin/bash
 # echo of all files in a directory

for file in *.dae
do
  name=${file%%[.]*}
  meshlabserver -i $file -o $name'.wrl' -om vn
  meshlabserver -i $file -o $name'.stl' -om vn
  meshlabserver -i $file -o $name'.obj' -om vn
  meshlabserver -i $file -o $name'_chull.obj' -om vn -s chull.mlx
  meshlabserver -i $file -o $name'_chull.wrl' -om vn -s chull.mlx
  meshlabserver -i $name'_chull.wrl' -o $name'_chull.wrl' -om vn -s quadric_edge_collapse_decimation.mlx
  meshlabserver -i $name'_chull.obj' -o $name'_chull.obj' -om vn -s quadric_edge_collapse_decimation.mlx
  #echo $name'.wrl' 
done
for file in *_chull.wrl
do
  name=${file%%[.]*}
  #echo $name'.wrl' 
done

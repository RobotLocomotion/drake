#!/bin/bash
 # echo of all files in a directory

for file in *.STL
do
  name=${file%%[.]*}
  meshlabserver -i $file -o $name'.wrl' -om vn
  meshlabserver -i $file -o $name'_chull.wrl' -om vn -s chull.mlx
  #echo $name'.wrl' 
done

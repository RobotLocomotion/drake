#!/bin/bash
 # echo of all files in a directory

for file in *.obj
do
  name=${file%%[.]*}
  meshlabserver -i $file -o $name'_1.obj' -om vn -s simplify.mlx
  #echo $name'.obj' 
done

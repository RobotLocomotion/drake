#!/bin/bash
 # echo of all files in a directory

for file in lj5.dae
do
  name=${file%%[.]*}
  meshlabserver -i $file -o $name'.stl' -om vn
  meshlabserver -i $file -o $name'.obj' -om vn
  echo $name
done

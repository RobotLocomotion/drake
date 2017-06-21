for file in *.stl;
  do meshlabserver -i $(basename $file .stl).stl -o $(basename $file .stl).obj;
done
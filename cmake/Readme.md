## FindEigen3, FindNumpy:

These .cmake files are invoked by cmake when we call:

	find_package(Eigen3)
	find_package(NumPy)

To use them, we have to add this folder to the `CMAKE_MODULE_PATH` variable in cmake. 

## SwigPython:

Defines a helpful function to build SWIG python bindings. 
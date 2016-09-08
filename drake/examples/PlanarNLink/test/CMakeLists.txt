add_matlab_test(NAME examples/PlanarNLink/test/bodyKinTest OPTIONAL bullet COMMAND bodyKinTest)
add_matlab_test(NAME examples/PlanarNLink/test/polyIK COMMAND polyIK REQUIRES spotless SIZE large)

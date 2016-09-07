drake_add_matlab_test(NAME examples/PlanarNLink/test/bodyKinTest OPTIONAL bullet COMMAND bodyKinTest)
drake_add_matlab_test(NAME examples/PlanarNLink/test/polyIK COMMAND polyIK REQUIRES spotless SIZE large)

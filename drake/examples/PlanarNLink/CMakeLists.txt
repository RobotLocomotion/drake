# add_matlab_test(NAME examples/PlanarNLink/runLQR COMMAND runLQR)  # FIXME
add_matlab_test(NAME examples/PlanarNLink/runPassive OPTIONAL bullet COMMAND runPassive)

add_subdirectory(test)

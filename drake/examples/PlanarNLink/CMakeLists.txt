# add_matlab_test(NAME examples/PlanarNLink/runLQR COMMAND runLQR)  # FIXME: see #3313
add_matlab_test(NAME examples/PlanarNLink/runPassive OPTIONAL bullet COMMAND runPassive)

add_subdirectory(test)

add_matlab_test(NAME examples/ZMP/CartTable.run COMMAND CartTable.run REQUIRES gurobi)
add_matlab_test(NAME examples/ZMP/CartTable2D.run COMMAND CartTable2D.run REQUIRES gurobi)

add_subdirectory(test)

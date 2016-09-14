drake_add_matlab_test(NAME examples/ZMP/CartTable.run OPTIONAL bullet gurobi COMMAND CartTable.run)
drake_add_matlab_test(NAME examples/ZMP/CartTable2D.run OPTIONAL bullet gurobi COMMAND CartTable2D.run)

add_subdirectory(test)

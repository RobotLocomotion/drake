add_matlab_test(NAME examples/CartPole/CartPoleEnergyControl.run REQUIRES spotless OPTIONAL lcm mosek sedumi COMMAND CartPoleEnergyControl.run)
add_matlab_test(NAME examples/CartPole/CartPoleEnergyShaping.run OPTIONAL lcm COMMAND CartPoleEnergyShaping.run)
add_matlab_test(NAME examples/CartPole/runHinf REQUIRES spotless OPTIONAL mosek sedumi COMMAND runHinf)
add_matlab_test(NAME examples/CartPole/runLQR OPTIONAL sedumi COMMAND runLQR)
add_matlab_test(NAME examples/CartPole/runPassive COMMAND runPassive)
add_matlab_test(NAME examples/CartPole/runSwingUp OPTIONAL snopt COMMAND runSwingUp)
add_matlab_test(NAME examples/CartPole/runTrajectorySwingUpAndBalance REQUIRES spotless OPTIONAL mosek sedumi snopt COMMAND runTrajectorySwingUpAndBalance)

add_subdirectory(test)

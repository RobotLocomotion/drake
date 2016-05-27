add_matlab_test(NAME examples/CartPole/CartPoleEnergyControl.run COMMAND CartPoleEnergyControl.run)
add_matlab_test(NAME examples/CartPole/CartPoleEnergyShaping.run COMMAND CartPoleEnergyShaping.run)
add_matlab_test(NAME examples/CartPole/runHinf COMMAND runHinf)
add_matlab_test(NAME examples/CartPole/runLQR COMMAND runLQR)
add_matlab_test(NAME examples/CartPole/runPassive COMMAND runPassive)
add_matlab_test(NAME examples/CartPole/runSwingUp COMMAND runSwingUp)
add_matlab_test(NAME examples/CartPole/runTrajectorySwingUpAndBalance COMMAND runTrajectorySwingUpAndBalance)

add_subdirectory(test)

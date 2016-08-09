add_matlab_test(NAME examples/CartPole/CartPoleEnergyControl.run COMMAND CartPoleEnergyControl.run REQUIRES mosek spotless)
add_matlab_test(NAME examples/CartPole/CartPoleEnergyShaping.run COMMAND CartPoleEnergyShaping.run)
add_matlab_test(NAME examples/CartPole/runHinf COMMAND runHinf REQUIRES mosek spotless)
add_matlab_test(NAME examples/CartPole/runLQR COMMAND runLQR)
add_matlab_test(NAME examples/CartPole/runPassive COMMAND runPassive)
add_matlab_test(NAME examples/CartPole/runSwingUp COMMAND runSwingUp REQUIRES snopt)
add_matlab_test(NAME examples/CartPole/runTrajectorySwingUpAndBalance COMMAND runTrajectorySwingUpAndBalance REQUIRES mosek snopt spotless)

add_subdirectory(test)

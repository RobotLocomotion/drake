add_matlab_test(NAME matlab/solvers/+drakeFunction/+euclidean/+test/normSquaredTest REQUIRES lcm OPTIONAL snopt COMMAND drakeFunction.euclidean.test.normSquaredTest)
add_matlab_test(NAME matlab/solvers/+drakeFunction/+euclidean/+test/signedDistanceToHyperplaneTest REQUIRES lcm OPTIONAL snopt COMMAND drakeFunction.euclidean.test.signedDistanceToHyperplaneTest)
add_matlab_test(NAME matlab/solvers/+drakeFunction/+euclidean/+test/smoothNormTest REQUIRES lcm OPTIONAL snopt COMMAND drakeFunction.euclidean.test.smoothNormTest)

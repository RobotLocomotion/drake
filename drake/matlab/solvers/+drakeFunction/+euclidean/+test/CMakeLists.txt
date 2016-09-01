add_matlab_test(NAME matlab/solvers/+drakeFunction/+euclidean/+test/normSquaredTest REQUIRES lcm libbot OPTIONAL snopt COMMAND drakeFunction.euclidean.test.normSquaredTest)
add_matlab_test(NAME matlab/solvers/+drakeFunction/+euclidean/+test/signedDistanceToHyperplaneTest REQUIRES lcm libbot OPTIONAL snopt COMMAND drakeFunction.euclidean.test.signedDistanceToHyperplaneTest)
add_matlab_test(NAME matlab/solvers/+drakeFunction/+euclidean/+test/smoothNormTest REQUIRES lcm libbot OPTIONAL snopt COMMAND drakeFunction.euclidean.test.smoothNormTest)

add_matlab_test(NAME solvers/+drakeFunction/+euclidean/+test/normSquaredTest COMMAND drakeFunction.euclidean.test.normSquaredTest REQUIRES snopt)
add_matlab_test(NAME solvers/+drakeFunction/+euclidean/+test/signedDistanceToHyperplaneTest COMMAND drakeFunction.euclidean.test.signedDistanceToHyperplaneTest REQUIRES snopt)
add_matlab_test(NAME solvers/+drakeFunction/+euclidean/+test/smoothNormTest COMMAND drakeFunction.euclidean.test.smoothNormTest REQUIRES snopt)

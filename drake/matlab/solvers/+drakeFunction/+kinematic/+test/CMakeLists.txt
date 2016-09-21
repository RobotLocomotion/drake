drake_add_matlab_test(NAME matlab/solvers/+drakeFunction/+kinematic/+test/relativePositionNonlinearProgramTest REQUIRES snopt OPTIONAL bullet COMMAND drakeFunction.kinematic.test.relativePositionNonlinearProgramTest)
drake_add_matlab_test(NAME matlab/solvers/+drakeFunction/+kinematic/+test/relativePositionTest OPTIONAL bullet COMMAND drakeFunction.kinematic.test.relativePositionTest)

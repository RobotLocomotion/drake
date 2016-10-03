drake_add_matlab_test(NAME matlab/solvers/trajectoryOptimization/test/contactImplicitBrick
  REQUIRES snopt COMMAND contactImplicitBrick SIZE large)

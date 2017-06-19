drake_add_matlab_test(NAME matlab/solvers/BMI/kinematics/test/testIKABB REQUIRES mosek spotless OPTIONAL bullet COMMAND testIKABB)
drake_add_matlab_test(NAME matlab/solvers/BMI/kinematics/test/testKinematics OPTIONAL bullet COMMAND testKinematics)
drake_add_matlab_test(NAME matlab/solvers/BMI/kinematics/test/testPolyhedronNoCollision REQUIRES mosek spotless COMMAND testPolyhedronNoCollision)

add_matlab_test(NAME examples/CubicPolynomialExample.findFixedPointTest OPTIONAL snopt COMMAND CubicPolynomialExample.findFixedPointTest)
add_matlab_test(NAME examples/CubicPolynomialExample.animate COMMAND CubicPolynomialExample.animate)
add_matlab_test(NAME examples/CubicPolynomialExample.sosExample REQUIRES mosek spotless COMMAND CubicPolynomialExample.sosExample)
add_matlab_test(NAME examples/CubicPolynomialExample.run REQUIRES spotless OPTIONAL mosek sedumi COMMAND CubicPolynomialExample.run)
add_matlab_test(NAME examples/DoubleIntegrator.runDircol OPTIONAL snopt COMMAND DoubleIntegrator.runDircol)
add_matlab_test(NAME examples/DoubleIntegrator.runDirtran OPTIONAL snopt COMMAND DoubleIntegrator.runDirtran)
add_matlab_test(NAME examples/DoubleIntegrator.runConvexDirtran OPTIONAL gurobi COMMAND DoubleIntegrator.runConvexDirtran)

add_matlab_test(NAME examples/GridWorld.runValueIteration COMMAND GridWorld.runValueIteration)
add_matlab_test(NAME examples/GridWorld.drawBoard COMMAND GridWorld.drawBoard)

add_matlab_test(NAME examples/LinearGaussianExample.run COMMAND LinearGaussianExample.run)

add_matlab_test(NAME examples/SineSys.runTaylor REQUIRES spotless OPTIONAL mosek sedumi COMMAND SineSys.runTaylor)
add_matlab_test(NAME examples/SineSys.runTrigPolyVectorFields REQUIRES spotless COMMAND SineSys.runTrigPolyVectorFields)
add_matlab_test(NAME examples/SineSys.run REQUIRES sedumi COMMAND SineSys.run)

add_matlab_test(NAME examples/VanDerPol.runDircol OPTIONAL snopt COMMAND VanDerPol.runDircol)
add_matlab_test(NAME examples/VanDerPol.phasePortrait COMMAND VanDerPol.phasePortrait)
add_matlab_test(NAME examples/VanDerPol.run REQUIRES spotless OPTIONAL mosek sedumi COMMAND VanDerPol.run)

add_matlab_test(NAME examples/commonLyapunovForLinearSystems REQUIRES mosek spotless COMMAND commonLyapunovForLinearSystems)

if(LONG_RUNNING_TESTS)
  add_matlab_test(NAME examples/DoubleIntegrator.runValueIteration COMMAND DoubleIntegrator.runValueIteration PROPERTIES TIMEOUT 1500)
  add_matlab_test(NAME examples/VanDerPol.particleDemo COMMAND VanDerPol.particleDemo PROPERTIES TIMEOUT 1500)
endif()

# recurse into all example subdirectories
# note: the GLOB workflow is typically not recommended due to the fact that CMake will not be able
# to detect changes, but we currently support users adding contributed examples by installing
# directly into examples/MY_EXAMPLE. See #1939.
file(GLOB children *)
foreach(child ${children})
  if(EXISTS ${child}/CMakeLists.txt)
    add_subdirectory(${child})
  endif()
endforeach()

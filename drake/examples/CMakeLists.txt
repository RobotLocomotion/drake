drake_add_matlab_test(NAME examples/CubicPolynomialExample.findFixedPointTest OPTIONAL snopt COMMAND CubicPolynomialExample.findFixedPointTest)
drake_add_matlab_test(NAME examples/CubicPolynomialExample.animate COMMAND CubicPolynomialExample.animate)
drake_add_matlab_test(NAME examples/CubicPolynomialExample.sosExample REQUIRES mosek spotless COMMAND CubicPolynomialExample.sosExample)
drake_add_matlab_test(NAME examples/CubicPolynomialExample.run REQUIRES spotless OPTIONAL mosek sedumi COMMAND CubicPolynomialExample.run)
drake_add_matlab_test(NAME examples/DoubleIntegrator.runDircol OPTIONAL snopt COMMAND DoubleIntegrator.runDircol)
drake_add_matlab_test(NAME examples/DoubleIntegrator.runDirtran OPTIONAL snopt COMMAND DoubleIntegrator.runDirtran)
drake_add_matlab_test(NAME examples/DoubleIntegrator.runConvexDirtran OPTIONAL gurobi COMMAND DoubleIntegrator.runConvexDirtran)

drake_add_matlab_test(NAME examples/GridWorld.runValueIteration COMMAND GridWorld.runValueIteration)
drake_add_matlab_test(NAME examples/GridWorld.drawBoard COMMAND GridWorld.drawBoard)

drake_add_matlab_test(NAME examples/LinearGaussianExample.run COMMAND LinearGaussianExample.run)

drake_add_matlab_test(NAME examples/SineSys.runTaylor REQUIRES spotless OPTIONAL mosek sedumi COMMAND SineSys.runTaylor)
drake_add_matlab_test(NAME examples/SineSys.runTrigPolyVectorFields REQUIRES spotless COMMAND SineSys.runTrigPolyVectorFields)
drake_add_matlab_test(NAME examples/SineSys.run REQUIRES sedumi COMMAND SineSys.run)

drake_add_matlab_test(NAME examples/VanDerPol.runDircol OPTIONAL snopt COMMAND VanDerPol.runDircol)
drake_add_matlab_test(NAME examples/VanDerPol.phasePortrait COMMAND VanDerPol.phasePortrait)
drake_add_matlab_test(NAME examples/VanDerPol.run REQUIRES spotless OPTIONAL mosek sedumi COMMAND VanDerPol.run)

drake_add_matlab_test(NAME examples/commonLyapunovForLinearSystems REQUIRES mosek spotless COMMAND commonLyapunovForLinearSystems)


drake_add_matlab_test(NAME examples/DoubleIntegrator.runValueIteration COMMAND DoubleIntegrator.runValueIteration SIZE large)
drake_add_matlab_test(NAME examples/VanDerPol.particleDemo COMMAND VanDerPol.particleDemo SIZE large)

add_executable(simple_continuous_time_system simple_continuous_time_system.cc)
target_link_libraries(simple_continuous_time_system
    drakeSystemAnalysis)
drake_add_test(NAME simple_continuous_time_system COMMAND simple_continuous_time_system)

# recurse into all example subdirectories
# Note: the GLOB workflow is typically not recommended due to the fact that
# CMake will not be able to detect changes, but we currently support users
# adding contributed examples by installing directly into
# examples/MY_EXAMPLE. See #1939.
file(GLOB children *)
foreach(child ${children})
  if(EXISTS ${child}/CMakeLists.txt)
    add_subdirectory(${child})
  endif()
endforeach()

# Convergence analysis for the unit test

The analitical value of the height at `t=0.6` is `z = -0.886544828396101`.
However this value is not directly used in the unit test since the analytical solution assumes ideal rigid bodies while Drake uses the [Kelvin–Voigt material](https://en.wikipedia.org/wiki/Kelvin%E2%80%93Voigt_material). Issue [#1978](https://github.com/RobotLocomotion/drake/issues/1978) comments on this.

These tests assesses the convergence of this mathematical model to the ideal rigid body model as the penetration stiffness is increased.

All tests are performed using RK2.

## Convergence tests in time step for a given penetration stiffness

### penetration_stiffness=5000.0

| delt | z(t=0.6) |
| ---- | -------- |
| 1.00e-3 | -0.90034789355041966 |
| 0.50e-3 | -0.90032496799219253 |
| 0.25e-3 | -0.9002952314885535  |
| 0.25e-4 | -0.90030032976790197 |

### penetration_stiffness=50000.0

| delt| z(t=0.6) |
| --- | -------- |
| 1.00e-3 | -0.88768033488430764 |
| 0.50e-3 | -0.88979062732228809 |
| 0.25e-3 | -0.88963237752708779 |
| 0.25e-4 | -0.88971795945811449 |
| 1.00e-4 | -0.88970906044044529 |

## Convergence tests in penetration stiffness for a given time step

### delt=0.25e-3

| stiffness |   z(t=0.6) |
| --- | -------- |
|  5.0e3 | -0.9002952314885535   |
|  5.0e4 | -0.88963237752708779	 |
|  5.0e5 | -0.88624295964943722	 |


### delt=0.25e-4

| stiffness  |  z(t=0.6) |
| --- | -------- |
 | 5.0e3 | -0.90030032976790197 |
 | 5.0e4 | -0.88971795945811449 |
 | 5.0e5 | -0.88742438149030523 |

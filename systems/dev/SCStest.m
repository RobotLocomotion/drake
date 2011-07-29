function SCStest

eclipse_root = '/Users/russt/Documents/workspace/';
javaaddpath([eclipse_root,'DoublePendulum/classes']);

r = com.yobotics.exampleSimulations.doublePendulum.DoublePendulumRobot();

p=SimulationConstructionSetRobot(r);



x=0*randn(4,1)
p.dynamics(0,x,zeros(2,1))

%x = r.getStateVectorArray()
%r.doDynamicsButDoNotIntegrate();
%xdot = r.getStateDerivativeVectorArray()


function SCStest

eclipse_root = '/Users/russt/Documents/workspace/';
javaaddpath([eclipse_root,'DoublePendulum/classes']);

r = com.yobotics.exampleSimulations.doublePendulum.DoublePendulumRobot();

javaaddpath([eclipse_root,'ThirdParty/ThirdPartyJars/filedrop.jar']);
%sim = com.yobotics.simulationconstructionset.Simulation(r,8192);
scs = com.yobotics.simulationconstructionset.SimulationConstructionSet(r);

return

p=SimulationConstructionSetRobot(r);



x=0*randn(4,1)
p.dynamics(0,x,zeros(2,1))

%x = r.getStateVectorArray()
%r.doDynamicsButDoNotIntegrate();
%xdot = r.getStateDerivativeVectorArray()


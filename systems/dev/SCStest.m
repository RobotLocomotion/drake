function SCStest

eclipse_root = '/Users/russt/Documents/workspace/';
javaaddpath([eclipse_root,'ThirdParty/ThirdPartyJars/Java3d/j3dcore.jar']);
javaaddpath([eclipse_root,'ThirdParty/ThirdPartyJars/Java3d/j3dutils.jar']);
javaaddpath([eclipse_root,'ThirdParty/ThirdPartyJars/Java3d/vecmath.jar']);
javaaddpath([eclipse_root,'SimulationConstructionSet/classes']);
javaaddpath([eclipse_root,'IHMCUtilities/classes']);
javaaddpath([eclipse_root,'Plotting/classes']);

javaaddpath([eclipse_root,'DoublePendulum/classes']);

r = com.yobotics.exampleSimulations.doublePendulum.DoublePendulumRobot();

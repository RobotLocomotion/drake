function x_struct = getSimulinkStateStructure(mdl)
% Note: this used to be inside DynamicalSystem.stateVectorToStructure, but
% having it there caused simulink to be loaded ( see 
% https://github.com/RobotLocomotion/drake/issues/1044 )

x_struct = Simulink.BlockDiagram.getInitialState(mdl);



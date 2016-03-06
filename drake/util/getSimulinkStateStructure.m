function x_struct = getSimulinkStateStructure(mdl)
% Note: this used to be inside DynamicalSystem.stateVectorToStructure, but
% having it there caused simulink to be loaded ( see 
% https://github.com/RobotLocomotion/drake/issues/1044 )

if (~strcmp(get_param(mdl,'SaveFormat'),'Structure'))
  if (~strcmp(get_param(mdl,'SimulationStatus'),'stopped'))
    feval(mdl,[],[],[],'term');
  end
  set_param(mdl,'SaveFormat','Structure');
end

x_struct = Simulink.BlockDiagram.getInitialState(mdl);



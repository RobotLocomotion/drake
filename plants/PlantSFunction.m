function lcmInput(block)
% Simulink block which encapsulates a simple Plant
% Dialog Paramaters:
%    object of type 'Plant'

setup(block);

end

function setup(block)

  plant = block.DialogPrm(1).Data;
  
  % Register number of ports
  block.NumInputPorts  = 1;
  block.InputPort(1).Dimensions = plant.num_u;
  block.InputPort(1).DatatypeID = 0; % double
  block.InputPort(1).Complexity  = 'Real';
  
  block.NumOutputPorts = 1;
  block.OutputPort(1).Dimensions = plant.num_x;
  block.OutputPort(1).DatatypeID = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  
  block.NumContStates = plant.num_x;
  
  % Setup port properties to be inherited or dynamic
%  block.SetPreCompInPortInfoToDynamic;
%  block.SetPreCompOutPortInfoToDynamic;
  
  % Register parameters
  block.NumDialogPrms     = 1;
  block.DialogPrmsTunable = {'Nontunable'};
  
  block.SampleTimes = [0,0];
      
  block.SimStateCompliance = 'DefaultSimState';
      
  block.RegBlockMethod('PostPropagationSetup',@DoPostPropSetup);
  block.RegBlockMethod('InitializeConditions',@InitialConditions);
  block.RegBlockMethod('Derivatives',@Derivatives);
  block.RegBlockMethod('Outputs', @Outputs);     % Required
  block.RegBlockMethod('Terminate', @Terminate); % Required
end

  function DoPostPropSetup(block)
    block.NumDworks = 0;
  end

  function InitialConditions(block)
    plant = block.DialogPrm(1).Data;
    block.ContStates.Data = getInitialState(plant);
  end
    
  function Derivatives(block)
    plant = block.DialogPrm(1).Data;
    u = block.InputPort(1).Data;
    u = min(max(u,plant.umin),plant.umax);
    block.Derivatives.Data = dynamics(plant,0,block.ContStates.Data,u);
  end

  function Outputs(block)
    block.OutputPort(1).Data = block.ContStates.Data;
  end

  function Terminate(block)
    % intentionally left empty
  end
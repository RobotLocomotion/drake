function DLSFunction(block)
% Simulink block which implements the Dynamical System
% Dialog Paramaters:
%    object of type 'DynamicalSystem'

mdlSetup(block);

end


function mdlSetup(block)
  sys = block.DialogPrm(1).Data;
  typecheck(sys,'DrakeSystem');

  % Register number of ports
  block.NumInputPorts  = 1;
  if (getNumInputs(sys)==0) % ok to take inputs, but just ignore them
    block.InputPort(1).Dimensions = -1;
  else
    block.InputPort(1).Dimensions = getNumInputs(sys);
  end
  block.InputPort(1).DatatypeID = 0; % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(1).DirectFeedthrough = isDirectFeedthrough(sys);

  block.NumOutputPorts = 1;
  block.OutputPort(1).Dimensions = getNumOutputs(sys);
  block.OutputPort(1).DatatypeID = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  
  block.NumContStates = getNumContStates(sys);
  
  % Setup port properties to be inherited or dynamic
  %  block.SetPreCompInPortInfoToDynamic;
  %  block.SetPreCompOutPortInfoToDynamic;
  
  % Register parameters
  block.NumDialogPrms     = 1;
  block.DialogPrmsTunable = {'Nontunable'};
  
  block.SampleTimes = [0,0];
  block.SimStateCompliance = 'DefaultSimState';
  
  %      block.RegBlockMethod('PostPropagationSetup',@mdlDoPostPropSetup);
  block.RegBlockMethod('InitializeConditions',@mdlInitialConditions);
  if (getNumContStates(sys)>0)
    block.RegBlockMethod('Derivatives',@mdlDerivatives);
  end
  if (getNumDiscStates(sys)>0)
    block.RegBlockMethod('Update',@mdlUpdate);
  end
  block.RegBlockMethod('Outputs', @mdlOutputs);     % Required
%  block.RegBlockMethod('ZeroCrossing', @mdlZeroCrossing);
  block.RegBlockMethod('Terminate', @mdlTerminate); % Required
end
    
%    function mdlDoPostPropSetup(block)
%      block.NumDworks = 0;
%    end

function mdlInitialConditions(block)
  sys = block.DialogPrm(1).Data;
  if (getNumStates(sys)>0)
    % todo: handle discrete states here, too.
    if (getNumDiscStates(sys)>0) error('discrete states not handled yet'); end
    block.ContStates.Data = getInitialState(sys);
  end
end

function mdlDerivatives(block)
  sys = block.DialogPrm(1).Data;
  t = block.CurrentTime;
  if (getNumDiscStates(sys)>0) error('discrete states not handled yet'); end
  x = block.ContStates.Data;
  u = min(max(block.InputPort(1).Data,sys.umin),sys.umax);
  block.Derivatives.Data = dynamics(sys,t,x,u);
end

function mdlUpdate(block)
  sys = block.DialogPrm(1).Data;
  t = block.CurrentTime;
  if (getNumDiscStates(sys)>0) error('discrete states not handled yet'); end
  x = block.ContStates.Data;
  u = min(max(block.InputPort(1).Data,sys.umin),sys.umax);
%  block.DiscStates.Data = update(sys,t,x,u);
  error('not handled yet');
end

function mdlOutputs(block)
  sys = block.DialogPrm(1).Data;
  t = block.CurrentTime;
  if (getNumDiscStates(sys)>0) error('discrete states not handled yet'); end
  x = block.ContStates.Data;
  u = min(max(block.InputPort(1).Data,sys.umin),sys.umax);
  block.OutputPort(1).Data = output(sys,t,x,u);
end

function mdlZeroCrossing(block)
  % todo: add zero crossings for input saturations if they exist
end

function mdlTerminate(block)
  % intentionally left empty
end
    

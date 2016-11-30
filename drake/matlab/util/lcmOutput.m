function lcmOutput(block)
% Simulink input block which subscribes to an LCM channel
%  takes three dialog parameters:
%    channelname (string)
%    signal dimension (int)
%    encoder function (function_handle)


setup(block);

function setup(block)

  checkDependency('lcm');

  % Register number of ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 0;
  
  % Setup port properties to be inherited or dynamic
  %block.SetPreCompOutPortInfoToDynamic;
  
  % Register parameters
  block.NumDialogPrms     = 4;
  block.DialogPrmsTunable = {'Nontunable','Nontunable','Nontunable','Tunable'};
    
  % Override output port properties
  block.InputPort(1).Dimensions       = block.DialogPrm(2).Data;
  block.InputPort(1).DatatypeID  = 0; % double
  block.InputPort(1).Complexity  = 'Real';
  
  block.SampleTimes = block.DialogPrm(4).Data;
  
  block.SimStateCompliance = 'DefaultSimState';
  
  block.RegBlockMethod('Outputs', @Outputs);     % Required
  block.RegBlockMethod('Terminate', @Terminate); % Required


function Outputs(block)
  publisher = block.DialogPrm(3).Data;
  t = block.CurrentTime;
  x = block.InputPort(1).Data;
  channel = block.DialogPrm(1).Data;
  publish(publisher,t,x,channel); 
  

function Terminate(block)

%end Terminate


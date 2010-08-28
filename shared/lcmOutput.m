function lcmOutput(block)
% Simulink input block which subscribes to an LCM channel
%  takes three dialog parameters:
%    channelname (string)
%    signal dimension (int)
%    encoder function (function_handle)


setup(block);

function setup(block)

  checkDependency('lcm_enabled');

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
  
  block.SampleTimes = [block.DialogPrm(4).Data,0];
  
  block.SimStateCompliance = 'DefaultSimState';
  
  lc = lcm.lcm.LCM.getSingleton();

  block.RegBlockMethod('Outputs', @(block)Outputs(block,lc));     % Required
  block.RegBlockMethod('Terminate', @Terminate); % Required


function Outputs(block,lc)
  encodeFcn = block.DialogPrm(3).Data;
  msg = encodeFcn(0,block.InputPort(1).Data); % todo: add simulation time here
  lc.publish(block.DialogPrm(1).Data,msg);
  

function Terminate(block)

%end Terminate


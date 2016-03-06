function lcmInput(block)
% Simulink input block which subscribes to an LCM channel
%  takes three dialog parameters:
%    channelname (string)
%    signal dimension (int)
%    decoder function (function_handle)


setup(block);

function setup(block)

  checkDependency('lcm');

  % Register number of ports
  block.NumInputPorts  = 0;
  block.NumOutputPorts = 1;
  
  % Setup port properties to be inherited or dynamic
  %block.SetPreCompOutPortInfoToDynamic;
  
  % Register parameters
  block.NumDialogPrms     = 4;
  block.DialogPrmsTunable = {'Nontunable','Nontunable','Nontunable','Tunable'};
    
  % Override output port properties
  block.OutputPort(1).SamplingMode = 'sample';
  block.OutputPort(1).Dimensions       = block.DialogPrm(2).Data;
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  
  block.SampleTimes = block.DialogPrm(4).Data;
  
  block.SimStateCompliance = 'DefaultSimState';

  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
  block.RegBlockMethod('Start', @Start);
  block.RegBlockMethod('Outputs', @Outputs);     % Required
  block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

function DoPostPropSetup(block)
  block.NumDworks = 0;


function Start(block)
  subscriber = block.DialogPrm(3).Data;
  channel = block.DialogPrm(1).Data;
  subscriber.subscribe(channel);


function Outputs(block)
  subscriber = block.DialogPrm(3).Data;
  x = getCurrentValue(subscriber);
  if isempty(x)
    block.OutputPort(1).Data = zeros(block.DialogPrm(2).Data,1);
  else
    block.OutputPort(1).Data = x;
  end
  

function Terminate(block)

%end Terminate


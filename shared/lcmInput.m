function lcmInput(block)
% Simulink input block which subscribes to an LCM channel
%  takes three dialog parameters:
%    channelname (string)
%    signal dimension (int)
%    decoder function (function_handle)


setup(block);

function setup(block)

  checkDependency('lcm_enabled');

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
  
  block.SampleTimes = [block.DialogPrm(4).Data,0];
  
  block.SimStateCompliance = 'DefaultSimState';
  
  lc = lcm.lcm.LCM.getSingleton();
  aggregator = lcm.lcm.MessageAggregator();
  aggregator.setMaxMessages(1);

  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
  block.RegBlockMethod('Start', @(block)Start(block,lc,aggregator));
  block.RegBlockMethod('Outputs', @(block)Outputs(block,aggregator));     % Required
  block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

function DoPostPropSetup(block)
  block.NumDworks = 0;


function Start(block,lc,aggregator)
  lc.subscribe(block.DialogPrm(1).Data,aggregator);


function Outputs(block,aggregator)
  if (aggregator.numMessagesAvailable()>0)
    umsg = getNextMessage(aggregator);
    decodeFcn = block.DialogPrm(3).Data;
    block.OutputPort(1).Data = decodeFcn(umsg);
  end
  

function Terminate(block)

%end Terminate


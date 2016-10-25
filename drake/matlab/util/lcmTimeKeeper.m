function lcmTimeKeeper(block)
% Simulink input block which subscribes to an LCM channel and watches the
% designated timestamp field of the designated message.  The output method
% will block until the lcm message timestamp catches up to the simulation
% time, therefore slowing simulink down to match the speed of the incoming
% LCM traffic.
%
% Dialog parameters:
%
% @param channel name of the channel to subscribe to
%
% @param MessageMonitor an instance of the lcm message type that should be watched
%
% @param timestamp_scale a scalar multiplier which relates the timestamp to
% the simulation time.  a timestamp_scale of 1 means the timestamp is in
% seconds.  a timestamp_scale of 1000 means the timestamp is in ms.
%
% @param sampletime

setup(block);

function setup(block)

  checkDependency('lcm');

  % Register number of ports
  block.NumInputPorts  = 0;
  block.NumOutputPorts = 0;
  
  % Setup port properties to be inherited or dynamic
  %block.SetPreCompOutPortInfoToDynamic;
  
  % Register parameters
  block.NumDialogPrms     = 4;
  block.DialogPrmsTunable = {'Nontunable','Nontunable','Nontunable','Tunable'};
      
  block.SampleTimes = [block.DialogPrm(4).Data,0];
  
  block.SimStateCompliance = 'DefaultSimState';

  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
  block.RegBlockMethod('Start', @Start);
  block.RegBlockMethod('Outputs', @Outputs);     % Required
  block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

function DoPostPropSetup(block)
  block.NumDworks = 1; 
  block.Dwork(1).Name = 'TimestampZero';
  block.Dwork(1).Dimensions = 1;
  block.Dwork(1).DataTypeID = 0;
  block.Dwork(1).Complexity = 'Real';
  block.Dwork(1).UsedAsDiscState = false;
  
function Start(block)
  typecheck(block.DialogPrm(1).Data,'char');  % channel
  mon = block.DialogPrm(2).Data;
  typecheck(mon,'drake.matlab.util.MessageMonitor'); 
  typecheck(block.DialogPrm(3).Data,'numeric'); % timstamp scale
  
  lc = lcm.lcm.LCM.getSingleton(); %('udpm://239.255.76.67:7667?ttl=1');
  lc.subscribe(block.DialogPrm(1).Data,mon);
  
  while (~mon.waitUntilTimestamp(0,1000)), end;  % block here until i get a message
  
  block.Dwork(1).Data = mon.getLastTimestamp();  % timestamp_zero
  

function Outputs(block)
  simtime = block.CurrentTime;
  mon = block.DialogPrm(2).Data;
  tscale = block.DialogPrm(3).Data;
  timestamp_zero = block.Dwork(1).Data;
  lcmtime = (mon.getLastTimestamp()-timestamp_zero)/tscale
  
  if (lcmtime - simtime > 2)
    timestamp_zero
    mon.getLastTimestamp()
    lcmtime
    simtime
    error('not keeping up with lcmtime');
    % todo: change this to a warning and warn only periodically
  end
  
  while (~mon.waitUntilTimestamp(tscale*simtime + timestamp_zero,1000)), end;

function Terminate(block)

%end Terminate


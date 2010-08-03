function timerobj = simulateLCM(obj,lcm_coder,dt,x0)
% Starts an LCM simulation node which listens for inputs and publishes state.
%   timerobj = simulateLCM(obj,lcm_coder[,dt,x0]) runs the dynamics as an LCM client,
%   listening for input messages and publishing regular state messages.
%   The timerobj that is returned will run forever unless it is stopped.
%   
%   The default value for dt is .01.
%   The default value for x0 is obj.getInitialState().


lc = lcm.lcm.LCM.getSingleton();
%aggregator = lcm.lcm.MessageAggregator();
%aggregator.setMaxMessages(1);  % make it a last-message-only queue

name = getRobotName(lcm_coder);
%lc.subscribe([name,'_input'],aggregator);

if (nargin<3) dt = 0.01; end
if (nargin<4) x0 = getInitialState(obj); end

if (dt<.001) 
  warning('Setting dt to .001, which is the minimum for matlab timers'); 
  dt = .001;
end

x = x0;

tic;
timerobj = timer('TimerFcn',{@timer_simulate,dt,name,lcm_coder,x},'ExecutionMode','fixedRate','Period',dt,'TasksToExecute',inf,'BusyMode','error','ErrorFcn','disp(''Timer error: Probably fell behind.  Consider increasing dt.'')');
start(timerobj); 


  function timer_simulate(timerobj,event,dt,name,lcm_coder,x)
    t=toc;
    msg = encodeState(lcm_coder,t,x);
    lc.publish([name,'_state'], msg);
  end

end

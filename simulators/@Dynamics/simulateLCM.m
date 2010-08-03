function simulateLCM(obj,lcm_coder,dt,x0)
% Starts an LCM simulation node which listens for inputs and publishes state.


%lc = lcm.lcm.LCM.getSingleton();
%aggregator = lcm.lcm.MessageAggregator();
%aggregator.setMaxMessages(1);  % make it a last-message-only queue

%name = getRobotName(lcm_coder);
%lc.subscribe([name,'_input'],aggregator);

if (dt<.01) warning('Setting dt to .01, which seems to be the minimum for matlab timers'); end
dt = max(dt,.01);  

tic;
ti = timer('TimerFcn',{@timer_draw,dt,name,lcm_coder},'ExecutionMode','fixedRate','Period',dt,'TasksToExecute',inf,'BusyMode','error');
start(ti); 
wait(ti);  % unfortunately, you can't try/catch a ctrl-c in matlab
delete(ti);


  function timer_simulate(timerobj,event,dt,name,lcm_coder)
    toc
%    t=toc;
%    msg = encodeLCMstate(lcm_coder,x);
%    msg.timestamp = t;
%    lc.publish([name,'_state'], msg);
  end

end
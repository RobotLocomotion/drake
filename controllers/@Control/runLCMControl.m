function timerobj = runLCMControl(obj,lcm_coder)
% Starts an LCM control node which listens for state and publishes inputs

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(1);  % make it a last-message-only queue

name=lcm_coder.getRobotName();
lc.subscribe([name,'_state'],aggregator);



if (obj.control_dt>0)
  % try to run on a clock
  timerobj = timer('TimerFcn',{@timer_control},'ExecutionMode','fixedRate','Period',obj.control_dt,'TasksToExecute',inf); %,'BusyMode','error','ErrorFcn','disp(''Timer error: Probably fell behind.  Consider increasing dt.'')');
  start(timerobj);
else
  % just run as fast as possible (and publish whenever there is new state
  % information to process)
  while true
    timer_control([],[]);
  end
end
  
  function timer_control(timerobj,event)
    
    if (aggregator.numMessagesAvailable()>0)
      xmsg = getNextMessage(aggregator);
      [x,t] = decodeState(lcm_coder,xmsg);

      u = obj.control(t,x);
      
      umsg = encodeInput(lcm_coder,t,u);
      lc.publish([name,'_input'], umsg);
    end
  end

end

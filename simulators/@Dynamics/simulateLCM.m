function runLCM(obj,robot_codec,dt,x0)

%lc = lcm.lcm.LCM.getSingleton();
%aggregator = lcm.lcm.MessageAggregator();
%aggregator.setMaxMessages(1);  % make it a last-message-only queue

%name = getRobotName(robot_codec);
%lc.subscribe([name,'_input'],aggregator);

if (dt<.01) warning('Setting dt to .01, which seems to be the minimum for matlab timers'); end
dt = max(dt,.01);  

tic;
ti = timer('TimerFcn',{@timer_draw,dt,name,robot_codec},'ExecutionMode','fixedRate','Period',dt,'TasksToExecute',inf,'BusyMode','error');
start(ti); 
wait(ti);  % unfortunately, you can't try/catch a ctrl-c in matlab
delete(ti);


  function timer_simulate(timerobj,event,dt,name,robot_codec)
    toc
%    t=toc;
%    msg = encodeLCMstate(robot_codec,x);
%    msg.timestamp = t;
%    lc.publish([name,'_state'], msg);
  end

end
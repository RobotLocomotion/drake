function timerobj = runLCMDynamics(obj,lcm_coder,dt,x0,visualizer)
% Starts an LCM simulation node which listens for inputs and publishes state.
%   timerobj = runLCMDynamics(obj,lcm_coder[,dt,x0,visualizer]) runs the dynamics as an LCM client,
%   listening for input messages and publishing regular state messages.
%   The timerobj that is returned will run forever unless it is stopped.
%   
%   The default value for dt is .01.
%   The default value for x0 is obj.getInitialState().


lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(1);  % make it a last-message-only queue

name=lcm_coder.getRobotName();
lc.subscribe([lower(name),'_input'],aggregator);

if (nargin<3) dt = 0.01; end
if (nargin<4) x0 = getInitialState(obj); end
if (nargin<5) visualizer=[]; end

if (dt<.001) 
  warning('Setting dt to .001, which is the minimum for matlab timers'); 
  dt = .001;
end

x = x0;

last_display_t = 0;

tic;
timerobj = timer('TimerFcn',{@timer_simulate},'ExecutionMode','fixedRate','Period',dt,'TasksToExecute',inf); %,'BusyMode','error','ErrorFcn','disp(''Timer error: Probably fell behind.  Consider increasing dt.'')');
start(timerobj); 


  function timer_simulate(timerobj,event)
    t=toc;
    dt = timerobj.InstantPeriod;
    
    if (~isnan(dt))
      if (aggregator.numMessagesAvailable()>0)
        umsg = getNextMessage(aggregator);
        u = decodeInput(lcm_coder,umsg);
      else
        u = getDefaultInput(obj);
      end

      x = x + dt*obj.dynamics(t,x,u);
      if (~isempty(visualizer) && t>(last_display_t+visualizer.display_dt))
        last_display_t = t;
        visualizer.draw(t,x,[]);
        drawnow;
      end
    end
    
    xmsg = encodeState(lcm_coder,t,x);
    lc.publish([lower(name),'_state'], xmsg);
  end

end

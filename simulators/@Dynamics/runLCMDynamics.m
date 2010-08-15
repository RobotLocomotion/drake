function timerobj = runLCMDynamics(obj,lcm_coder,options)
% Starts an LCM simulation node which listens for inputs and publishes state.
%   timerobj = runLCMDynamics(obj,lcm_coder,options) runs the dynamics as an LCM client,
%   listening for input messages and publishing regular state messages.
%   The timerobj that is returned will run forever unless it is stopped.
%   
%   The default value for dt is .01.
%   The default value for x0 is obj.getInitialState().
%   Input values of [] for dt or x0 will also result in using the defaults.

checkDependency('lcm_enabled');

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(1);  % make it a last-message-only queue

name=lcm_coder.getRobotName();
lc.subscribe([lower(name),'_u'],aggregator);

if (nargin<3) options = struct(); end
if (~isfield(options,'dt')) options.dt = 0.01; end
if (~isfield(options,'x0')) options.x0 = getInitialState(obj); end
if (~isfield(options,'visualizer')) options.visualizer = []; end
if (~isfield(options,'T')) options.T = inf; end

if (options.dt<.001) 
  warning('Setting dt to .001, which is the minimum for matlab timers'); 
  options.dt = .001;
end

x = options.x0;

last_display_t = 0;

tic;
timerobj = timer('TimerFcn',{@timer_simulate},'ExecutionMode','fixedRate','Period',options.dt,'TasksToExecute',inf); %,'BusyMode','error','ErrorFcn','disp(''Timer error: Probably fell behind.  Consider increasing dt.'')');
start(timerobj); 


  function timer_simulate(timerobj,event)
    t=toc;
    dt = timerobj.InstantPeriod;
    if (t>options.T) stop(timerobj); end
    
    if (~isnan(dt))
      if (aggregator.numMessagesAvailable()>0)
        umsg = getNextMessage(aggregator);
        u = decodeU(lcm_coder,umsg);
      else
        u = getDefaultInput(obj);
      end

      x = x + dt*obj.dynamics(t,x,u);
      if (~isempty(options.visualizer) && t>(last_display_t+options.visualizer.display_dt))
        last_display_t = t;
        if (options.visualizer.draw(t,x,[])), stop(timerobj); end
        drawnow;
      end
    end
    
    xmsg = encodeX(lcm_coder,t,x);
    lc.publish([lower(name),'_xhat'], xmsg);
  end

end

function timerobj = runLCMVisualizer(obj,lcm_coder)
% Starts an LCM visualizer node which listens for state and draws

checkDependency('lcm_enabled');

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(1);  % make it a last-message-only queue

name=lcm_coder.getRobotName();
lc.subscribe([lower(name),'_xhat'],aggregator);

% just draw as fast as I can...
while true
  xmsg = getNextMessage(aggregator,1000);
  if (~isempty(xmsg))
    [x,t] = decodeX(lcm_coder,xmsg);
    draw(obj,t,x,[]);
  end
end

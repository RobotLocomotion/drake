function timerobj = runLCMControl(obj,lcm_coder)
% Starts an LCM control node which listens for state and publishes inputs

checkDependency('lcm_enabled');

lc = lcm.lcm.LCM.getSingleton(); %('udpm://239.255.76.67:7667?ttl=1');

%lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(1);  % make it a last-message-only queue

name=lcm_coder.getRobotName();
lc.subscribe([lower(name),'_xhat'],aggregator);

% just run as fast as possible 
while true
  xmsg = getNextMessage(aggregator,1000);
  if (~isempty(xmsg))
    [x,t] = decodeX(lcm_coder,xmsg);
    
    u = obj.control(t,x);
    
    umsg = encodeU(lcm_coder,t,u);
    lc.publish([lower(name),'_u'], umsg);
  end
end


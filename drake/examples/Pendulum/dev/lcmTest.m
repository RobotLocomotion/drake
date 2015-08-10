
lc = lcm.lcm.LCM.getSingleton();
mon = drake.util.MessageMonitor(drake.examples.Pendulum.lcmt_pendulum_u,'timestamp');
lc.subscribe('PendulumInput',mon);

while(1);
  
  data = getNextMessage(mon,10000);
  if ~isempty(data)
    msg = drake.examples.Pendulum.lcmt_pendulum_u(data);
    fprintf(1,'%f\n',msg.timestamp/1000);
  end
end
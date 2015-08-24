
lc = lcm.lcm.LCM.getSingleton();
mon = drake.util.MessageMonitor(drake.lcmt_drake_signal,'timestamp');
lc.subscribe('PendulumInput',mon);

while(1);
  
  data = getNextMessage(mon,10000);
  if ~isempty(data)
    msg = drake.lcmt_drake_signal(data);
    fprintf(1,'%f\n',msg.timestamp/1000);
  end
end
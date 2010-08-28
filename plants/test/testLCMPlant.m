function testLCMPlant

load robotlib_config;

p = path;

addpath([conf.root,'/examples/Pendulum']);

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('pendulum_xhat',aggregator);

runLCMPlant(PendulumPlant,PendulumLCMCoder,struct('tspan',[0 5]));

if (aggregator.numMessagesAvailable()<1)
  error('looks like i''m not receiving LCM dynamics messages');
end

path(p);
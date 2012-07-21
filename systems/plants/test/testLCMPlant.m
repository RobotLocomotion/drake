function testLCMPlant

checkDependency('lcm_enabled')

load drake_config;

p = addpath([conf.root,'/examples/Pendulum']);

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('PendulumState',aggregator);

runLCM(PendulumPlant,[],struct('tspan',[0 5]));

if (aggregator.numMessagesAvailable()<1)
  error('looks like i''m not receiving LCM dynamics messages');
end

path(p);

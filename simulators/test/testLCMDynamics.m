function testLCMDynamics

load robotlib_config;

p = path;

addpath([conf.root,'/examples/Pendulum']);

%[s,w] = system(['matlab -nosplash -nodesktop -r "cd ', conf.root,'/examples/Pendulum; h=runLCMDynamics(PendulumDynamics,PendulumLCMCoder,struct(''T'',30));set(h,''StopFcn'',''exit'')" &'])
h=runLCMDynamics(PendulumDynamics,PendulumLCMCoder,struct('T',20));

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('pendulum_xhat',aggregator);

pause(30);

if (aggregator.numMessagesAvailable()<1)
  error('looks like i''m not receiving LCM dynamics messages');
end

path(p);
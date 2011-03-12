function testLCMControl

checkDependency('lcm_enabled');

load robotlib_config;

p = addpath([conf.root,'/examples/Pendulum']);

lc = lcm.lcm.LCM.getSingleton();
a = lcm.lcm.MessageAggregator();
coder = PendulumLCMCoder;
lc.subscribe('pendulum_u',a);

job = batch('runPendLCMControl','Workspace',struct());

waitForState(job,'running');

while (a.numMessagesAvailable()==0)
  xmsg = encodeX(coder,0,randn(2,1));
  lc.publish([lower(coder.getRobotName()),'_xhat'], xmsg);
  pause(1)

  errmsgs = get(job.Tasks, {'ErrorMessage'});
  nonempty = ~cellfun(@isempty, errmsgs);
  if (any(nonempty))
    celldisp(errmsgs(nonempty))
    destroy(job);
    path(p);
    error('found LCM error');
  end
end

% if I got here, then I got a response to my state query
destroy(job);
path(p);

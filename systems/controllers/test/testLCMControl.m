function testLCMControl

% NOTEST  
% this one is not playing nicely with matlabpool (at least not consistently).  

checkDependency('lcm_enabled');

load drake_config;

p = addpath([conf.root,'/examples/Pendulum']);

lc = lcm.lcm.LCM.getSingleton();
a = lcm.lcm.MessageAggregator();
coder = PendulumLCMCoder;
lc.subscribe('pendulum_u',a);

if (matlabpool('size')>0), matlabpool('close'); end
job = batch('runPendLCMControl','Workspace',struct(),'Matlabpool',0);

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

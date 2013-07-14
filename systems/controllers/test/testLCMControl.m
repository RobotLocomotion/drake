function testLCMControl

% NOTEST  - the path and javaclasspath on the build server doesn't work
% yet.  disabling this test for now.

% this one is not playing nicely with matlabpool (at least not consistently).  

checkDependency('lcm');

p = addpath([getDrakePath,'/examples/Pendulum']);

lc = lcm.lcm.LCM.getSingleton();
a = lcm.lcm.MessageAggregator();
lc.subscribe('PendulumInput',a);

if (matlabpool('size')>0), matlabpool('close'); end
%job = batch('javaaddpath(''drake.jar'');runPendLCMControl','Workspace',struct(),'matlabpool',0,...
%  'AdditionalPaths',regexp(path,':','split'), ...  % add the entire path
%  'AttachedFiles',[getDrakePath(),'/drake.jar']);
job = batch('runPendLCMControl','Workspace',struct(),'matlabpool',0);

%waitForState(job,'running');
wait(job,'running');

fr = PendulumState;
while (a.numMessagesAvailable()==0)
  xmsg = encode(fr,0,randn(2,1));
  lc.publish('PendulumState', xmsg);
  pause(1)

  errmsgs = get(job.Tasks, {'ErrorMessage'});
  nonempty = ~cellfun(@isempty, errmsgs);
  if (any(nonempty))
    celldisp(errmsgs(nonempty))
    %destroy(job);
    delete(job);
    path(p);
    error('found LCM error');
  end
end

% if I got here, then I got a response to my state query
%destroy(job);
delete(job);
path(p);

end
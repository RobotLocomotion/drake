function testLCMControl

% NOTEST  - the path and javaclasspath on the build server doesn't work
% yet.  disabling this test for now.

% this one is not playing nicely with matlabpool (at least not consistently). 
% if you get a message about being queued forever, you might have to
% restart your matlabpool, e.g. with:
% if (exist('matlabpool','file') && matlabpool('size')>0), matlabpool('close'); end

% todo: reimplement it without the parallel toolbox

checkDependency('lcm');
checkDependency('distcomp');

tmppath = addpathTemporary([getDrakePath,'/examples/Pendulum']);

lc = lcm.lcm.LCM.getSingleton();
a = lcm.lcm.MessageAggregator();
lc.subscribe('PendulumInput',a);

job = batch(['run(''',fullfile(getDrakePath,'addpath_drake.m'),''');runPendLCMControl'],'Workspace',struct(),'CurrentFolder',pwd);
%job = batch('runPendLCMControl','Workspace',struct());

wait(job,'running',90);

fr = PendulumState;
while (a.numMessagesAvailable()==0)
  fr.publish(0,randn(2,1),'PendulumState');
  pause(1)

  errmsgs = get(job.Tasks, {'ErrorMessage'});
  nonempty = ~cellfun(@isempty, errmsgs);
  if (any(nonempty))
    celldisp(errmsgs(nonempty))
    diary(job)
    %destroy(job);
    delete(job);
    error('error on remote job');
  end
end

% if I got here, then I got a response to my state query
%destroy(job);
delete(job);
path(p);

end
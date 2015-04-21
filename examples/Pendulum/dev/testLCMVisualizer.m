function testLCMVisualizer

% NOTEST - have to update this (probably to not use the parallel toolbox)

tmpdir = addpathTemporary([getDrakePath,'/examples/Pendulum']);
job1 = batch('runPendulum','Workspace',struct(),'CaptureDiary',true);

%ok = waitForState(job1,'running');
ok = wait(job1,'running');
if (~ok) error('failed'); end

runLCM(PendulumVisualizer,[],struct('tspan',[0 5]))

diary(job1);
%destroy(job1);
delete(job1);


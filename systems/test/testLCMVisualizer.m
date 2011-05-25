function testLCMVisualizer

% NOTEST

load robotlib_config;

p = path;

addpath([conf.root,'/examples/Pendulum']);

job1 = batch('runPendulum','Workspace',struct(),'CaptureDiary',true);

ok = waitForState(job1,'running',10);
if (~ok) error('failed'); end

runLCMVisualizer(PendulumVisualizer,PendulumLCMCoder,struct('tspan',[0 2]))

diary(job1);
destroy(job1);

path(p);

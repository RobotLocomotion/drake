function testLCMVisualizer

load robotlib_config;

p = path;

addpath([conf.root,'/examples/Pendulum']);

job1 = batch('runPendulum','Workspace',struct(),'CaptureDiary',true);

waitForState(job1,'running');

runLCMVisualizer(PendulumVisualizer,PendulumLCMCoder,struct('tspan',[0 2]))

diary(job1);
destroy(job1);

path(p);

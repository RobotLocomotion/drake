function runPendulum

load drake_config;
addpath([conf.root,'/examples/Pendulum']);

runLCM(PendulumPlant,[],struct('tspan',[0 2]));


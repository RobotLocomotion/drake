
load drake_config;
p = path;
addpath([conf.root,'/examples/Pendulum']);

runLCMControl(PendulumLQR(PendulumPlant),PendulumLCMCoder);

% NOTEST

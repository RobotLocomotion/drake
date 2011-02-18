
% NOTEST

load robotlib_config;
addpath([conf.root,'/examples/Pendulum']);

runLCMPlant(PendulumPlant,PendulumLCMCoder);


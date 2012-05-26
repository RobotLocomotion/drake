
% NOTEST

load drake_config;
addpath([conf.root,'/examples/Pendulum']);

runLCMPlant(PendulumPlant,PendulumLCMCoder);


function runPendulum

addpath([getDrakePath,'/examples/Pendulum']);

runLCM(PendulumPlant,[],struct('tspan',[0 2]));


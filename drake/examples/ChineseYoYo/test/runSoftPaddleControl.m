function runSoftPaddleControl
% Test the Controlled Soft Paddle Robot
curpath = pwd;
cd('..')
time = 1;
showGraphs = false;
SoftPaddleControl.run(time,showGraphs);
cd(curpath)
end
function runSoftPaddleHybrid
% Test the Soft Paddle Hybrid Robot
curpath = pwd;
cd('..');
SoftPaddleHybrid.runPassive();
cd(curpath)
end
function runSoftPaddleHybrid
% Test the Soft Paddle Hybrid Robot
addpathTemporary([getDrakePath,'/examples/ChineseYoYo']);
SoftPaddleHybrid.runPassive();
end
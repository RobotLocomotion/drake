function runAtlasWalkingTestMex()

oldpath = addpath(fullfile(getDrakePath,'examples','Atlas'));
runAtlasWalking(2,0,0,[0.5;0;0;0;0;0]);
path = oldpath;

end


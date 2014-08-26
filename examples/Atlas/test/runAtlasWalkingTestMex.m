function runAtlasWalkingTestMex()

oldpath = addpath(fullfile(getDrakePath,'examples','Atlas'));
finishup = onCleanup(@() path(oldpath));
runAtlasWalking(2,0,0,[0.5;0;0;0;0;0]);

end


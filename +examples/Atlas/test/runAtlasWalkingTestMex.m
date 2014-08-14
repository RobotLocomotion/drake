function runAtlasWalkingTestMex()

addpath(fullfile(getDrakePath,'examples','Atlas'));
runAtlasWalking(2,0,0,[0.5;0;0;0;0;0]);
rmpath(fullfile(getDrakePath,'examples','Atlas'));

end


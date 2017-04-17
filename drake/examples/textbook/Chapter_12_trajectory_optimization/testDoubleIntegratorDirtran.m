function testDoubleIntegratorDirtran

% note: requires Drake ver >= 0.9.7

cd(fullfile(getDrakePath,'examples'));
DoubleIntegrator.runDirtran;


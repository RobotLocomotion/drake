function testDoubleIntegratorDirtran

% note: requires Drake ver >= 0.9.7

cd(fullfile(getDrakePath,'examples'));
DoubleIntegrator.runDirtran;

% make sure you take a look at the code!
edit('DoubleIntegrator.runDirtran')


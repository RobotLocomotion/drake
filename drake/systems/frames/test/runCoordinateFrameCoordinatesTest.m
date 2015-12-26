function runCoordinateFrameCoordinatesTest

% This is really hacky. The reason this is in place is so that I can run
% the tests directly from Drake's unitTest framework. The key problem here
% seems to be that I'm trying to use assertEqual. assertEqual does not
% raise an error when a test fails, which is what Drake seems to be relying
% on when it determines whether a test has passed or failed. This
% necessitates manually checking whether a test has passed.

% I'm preferring assertEqual here since the results of a test failure
% for string comparison when using Matlab's assert and strcmp or when using
% Drake's checkequal are not informative.

% Please change this code if you know of a better way todo this.

% save test results from running tests
testResults = run(CoordinateFrameCoordinatesTest);
% check whether all tests were passed
didAllPass = all(arrayfun(@(testResult) testResult.Passed, testResults));

assert(didAllPass);
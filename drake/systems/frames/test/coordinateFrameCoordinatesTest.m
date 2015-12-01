function coordinateFrameCoordinatesTest

% this is really hacky - I would rather be able to run a class-based test
% directly.

testResults = run(CoordinateFrameCoordinatesTest);
didAllPass = all(arrayfun(@(testResult) testResult.Passed, testResults));

assert(didAllPass)
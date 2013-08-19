function paramExtractFromURDF

r1 = RigidBodyManipulator('../Acrobot.urdf');
r2 = RigidBodyManipulator('AcrobotWParams.urdf');

fr = getParamFrame(r2)

% NOTEST (not ready yet!)
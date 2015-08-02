function testAngularvel2quatdotMatrix

axis = randn(3, 1);
axis = axis / norm(axis);
angle = 2 * pi * (rand - 0.5);
q = axis2quat([axis; angle]);
valuecheck(eye(3), quatdot2angularvelMatrix(q) * angularvel2quatdotMatrix(q));

end
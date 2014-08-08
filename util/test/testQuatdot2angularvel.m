function testQuatdot2angularvel()

testAgainstInverse();

end

function testAgainstInverse()
nTests = 100;
for i = 1 : nTests
  q = randn(4, 1);
  q = q / norm(q);
  
  omega = randn(3, 1);
  qd = 1/2 * quatProduct([0; omega], q);
  omegaBack = quatdot2angularvel(q, qd);
  valuecheck(omegaBack, omega, 1e-12);
end
end
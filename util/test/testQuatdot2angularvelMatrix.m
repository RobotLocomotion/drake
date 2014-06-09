function testQuatdot2angularvelMatrix()

testGradient();

end

function testGradient()
nTests = 100;
for i = 1 : nTests
  q = randn(4, 1);
%   q = q / norm(q);
  [~, dM] = quatdot2angularvelMatrix(q);
  option.grad_method = 'taylorvar';
  [~, dM_geval] = geval(1, @quatdot2angularvelMatrix, q, option);
  valuecheck(dM_geval, dM, 1e-12);
end
end
function testQuatdot2angularvelMatrix()

testGradient();

end

function testGradient()
nTests = 100;
for i = 1 : nTests
  q = randn(4, 1);
%   q = q / norm(q);
  option.grad_method = {'taylorvar', 'user'};
  [~, ~] = geval(1, @quatdot2angularvelMatrix, q, option);
end
end
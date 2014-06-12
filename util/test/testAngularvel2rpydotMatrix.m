function testAngularvel2rpydotMatrix()

testGradient();

end

function testGradient()
nTests = 100;
for i = 1 : nTests
  rpy = randn(3, 1);
  option.grad_method = {'taylorvar', 'user'};
  [~, ~, ~] = geval(1, @angularvel2rpydotMatrix, rpy, option);
end
end
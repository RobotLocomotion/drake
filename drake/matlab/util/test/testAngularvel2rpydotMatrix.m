function testAngularvel2rpydotMatrix()

testGradient();

end

function testGradient()

option.grad_method = {'taylorvar', 'user'};
for i=1:100
  rpy = uniformlyRandomNonsingularRPY();
  [~, ~, ~] = geval(1, @angularvel2rpydotMatrix, rpy, option);
end

end
function testAngularvel2rpydotMatrix()

testGradient();

end

function testGradient()
n_tests = 100;
test_number = 1;
option.grad_method = {'taylorvar', 'user'};
while test_number < n_tests
  rpy = randn(3, 1);
  if abs(cos(rpy(2))) > 1e-2
    [~, ~, ~] = geval(1, @angularvel2rpydotMatrix, rpy, option);
    test_number = test_number + 1;
  end
end
end
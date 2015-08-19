function testMotionSubspace()

testGradients();

end

function testGradients()
option.grad_method = {'user', 'taylorvar'};

for i = 1 : 10
  body.floating = 0;
  body.joint_axis = randn(3, 1);
  q = randn;
  body.pitch = 0;
  [~, ~] = geval(1, @(q) motionSubspace(body, q), q, option);
  body.pitch = inf;
  [~, ~] = geval(1, @(q) motionSubspace(body, q), q, option);
  body.pitch = randn;
  [~, ~] = geval(1, @(q) motionSubspace(body, q), q, option);
  
  body.floating = 1;
  q = [uniformlyRandomRPY(); randn(3, 1)];
  [~, ~] = geval(1, @(q) motionSubspace(body, q), q, option);
  
  body.floating = 2;
  q = [uniformlyRandomQuat(); randn(3, 1)];
  [~, ~] = geval(1, @(q) motionSubspace(body, q), q, option);
end

end
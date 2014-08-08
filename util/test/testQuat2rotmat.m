function testQuat2rotmat()

for i = 1 : 100
  q = randn(4, 1);
%   q = q / norm(q);
  [~, dR] = quat2rotmat(q);
  
  option.grad_method = 'taylorvar';
  [~, dR_geval] = geval(1, @quat2rotmat, q, option);
  
  valuecheck(dR_geval, dR);
end
end
function testRotmat2quat()

for i = 1 : 100
  % test gradient
  q = randn(4, 1);
  q = q / norm(q);
  R = quat2rotmat(q);
  nq = randi([1 10]);
  
  dR = randn(numel(R), nq);
  [~, dq] = rotmat2quat(R, dR);

  option.grad_method = 'taylorvar';
  [~, dqdR_geval] = geval(1, @rotmat2quat, R, option);
  dq_geval = dqdR_geval * dR;
  
  valuecheck(dq, dq_geval);
end

end
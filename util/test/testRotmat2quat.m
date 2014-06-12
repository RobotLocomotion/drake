function testRotmat2quat()

for i = 1 : 100
  q_in = randn(4, 1);
  q_in = q_in / norm(q_in);
  R = quat2rotmat(q_in);
  nq = randi([1 10]);
  
  dR = randn(numel(R), nq);
  [q, dq] = rotmat2quat(R, dR);
  q_diff = quatDiff(q_in, q);
  a_diff = quat2axis(q_diff);
  angle_diff = angleDiff(0, a_diff(4));
  valuecheck(0, angle_diff, 1e-10);

  option.grad_method = 'taylorvar';
  [~, dqdR_geval] = geval(1, @rotmat2quat, R, option);
  dq_geval = dqdR_geval * dR;
  
  valuecheck(dq, dq_geval);
end

end
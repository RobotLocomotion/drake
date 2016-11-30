function testRotmat2rpy()

for i = 1 : 100
  % test gradient: 2D rotation matrix case
  angle = rand * 2 * pi - pi;
  R = rotmat(angle);
  
  nq = randi([1 10]);
  dR = randn(numel(R), nq);
  
  [~, drpy] = rotmat2rpy(R, dR);
  
  checkAgainstGevalGradient(drpy, R, dR);
end


for i = 1 : 100
  % test gradient: 3D rotation matrix case
  axis = randn(3, 1);
  axis = axis / norm(axis);
  angle = rand * 2 * pi - pi;
  R = axis2rotmat([axis; angle]);
  
  nq = randi([1 10]);
  dR = randn(numel(R), nq);
  
  [~, drpy] = rotmat2rpy(R, dR);
  
  checkAgainstGevalGradient(drpy, R, dR);
end

end

function checkAgainstGevalGradient(drpy, R, dR)
option.grad_method = 'taylorvar';
[~, drpydR_geval] = geval(1, @rotmat2rpy, R, option);
drpy_geval = drpydR_geval * dR;

valuecheck(drpy, drpy_geval);
end
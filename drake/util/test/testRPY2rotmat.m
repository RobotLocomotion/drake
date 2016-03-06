function testRPY2rotmat
% test the gradient of rpy2rotmat
for i = 1:100
  rpy = uniformlyRandomNonsingularRPY();
  [~,dR,ddR] = rpy2rotmat(rpy);
  [~,dR_var,ddR_var] = geval(@rpy2rotmat,rpy,struct('grad_method','taylorvar'));
  valuecheck(dR,dR_var,1e-4);
  valuecheck(ddR,ddR_var,1e-4);
end
end
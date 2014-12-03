function testRPY2rotmat
% test the gradient of rpy2rotmat
for i = 1:100
  rpy = uniformlyRandomNonsingularRPY();
  [~,dR] = rpy2rotmat(rpy);
  [~,dR_numeric] = geval(@rpy2rotmat,rpy,struct('grad_method','numerical'));
  valuecheck(dR,dR_numeric,1e-4);
end
end
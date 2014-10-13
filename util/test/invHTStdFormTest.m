function invHTStdFormTest
  rpy = 2*pi*rand(3,1)-pi;
  p = rand(3,1);
  makeT = @(euler_angles,pos) [rpy2rotmat(euler_angles),pos;zeros(1,3),1];
  [T,dT,ddT] = geval(makeT,rpy,p,struct('grad_method','taylorvar'));

  makeInvT = @(euler_angles,pos) invHTstd(makeT(euler_angles,pos),dT,ddT);

  options.grad_method = {'taylorvar','user'};
  [invT, dinvT, ddinvT] = geval(makeInvT,rpy,p,options);
end
